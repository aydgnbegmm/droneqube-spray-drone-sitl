#!/usr/bin/env python3
"""
Spray drone SITL test: takeoff -> hover -> kare rota -> LAND, telemetri CSV kaydeder.

Kullanim:
  python3 run_scenario.py --label empty --conn udp:127.0.0.1:14560
  python3 run_scenario.py --label full  --conn udp:127.0.0.1:14560 --set MOT_HOVER_LEARN=0 MOT_THST_HOVER=0.55

Onkosul: Gazebo modelindeki kutle (ve gerekirse motor itkisi) elle degistirilmis olmali.
Bu script kutleyi DEGISTIRMEZ; sadece etiketler ve kaydeder.
"""
import argparse, csv, math, time, json
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--label", required=True, help="orn. empty / full")
ap.add_argument("--mass", type=float, default=None, help="Gazebo'da ayarladiginiz toplam kutle (kg), sadece kayit icin")
ap.add_argument("--conn", default="udp:127.0.0.1:14560")
ap.add_argument("--alt", type=float, default=5.0)
ap.add_argument("--side", type=float, default=20.0, help="kare kenari (m)")
ap.add_argument("--hover_s", type=float, default=15.0)
ap.add_argument("--set", nargs="*", default=[], help="PARAM=deger")
ap.add_argument("--out", default=None)
args = ap.parse_args()
out = args.out or f"log_{args.label}.csv"

m = mavutil.mavlink_connection(args.conn)
m.wait_heartbeat()
print("Heartbeat alindi, sys", m.target_system)

def set_param(name, val):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(val),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.3)

def get_param(name):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < 2.5:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and msg.param_id.rstrip("\x00") == name:
            return round(float(msg.param_value), 4)
    return None

for kv in args.set:
    k, v = kv.split("=")
    set_param(k, v)

used = {p: get_param(p) for p in ["MOT_THST_HOVER", "MOT_HOVER_LEARN", "ATC_ACC_R_MAX",
                                   "WPNAV_ACCEL", "WPNAV_SPEED", "PSC_ACCZ_P", "PSC_ACCZ_I",
                                   "INS_HNTCH_ENABLE", "MOT_THST_EXPO"]}
json.dump({"label": args.label, "mass_kg": args.mass, "params": used}, open(out.replace(".csv", "_meta.json"), "w"), indent=1)
print("Parametreler:", used)

# istek: mesaj hizlari
for mid, hz in [(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20),
                (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20),
                (mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 10),
                (mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION, 5),
                (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2)]:
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0)

state = dict(t0=time.time(), phase="init", sp=(0.0, 0.0, 0.0))
last = {}
rows = []
events = []

def poll():
    """Tum bekleyen mesajlari isle, bir satir kaydet."""
    while True:
        msg = m.recv_match(blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        if t == "BAD_DATA":
            continue
        if t == "STATUSTEXT":
            events.append((time.time() - state["t0"], msg.text))
            print("[FC]", msg.text)
        last[t] = msg
    lp, at, so, vb, ek = (last.get(k) for k in
        ["LOCAL_POSITION_NED", "ATTITUDE", "SERVO_OUTPUT_RAW", "VIBRATION", "EKF_STATUS_REPORT"])
    if lp and at:
        rows.append([round(time.time() - state["t0"], 3), state["phase"],
                     lp.x, lp.y, lp.z, lp.vx, lp.vy, lp.vz,
                     math.degrees(at.roll), math.degrees(at.pitch), math.degrees(at.yaw),
                     *state["sp"],
                     *(getattr(so, f"servo{i}_raw", 0) if so else 0 for i in range(1, 5)),
                     vb.vibration_x if vb else 0, vb.vibration_y if vb else 0, vb.vibration_z if vb else 0,
                     vb.clipping_0 if vb else 0, ek.flags if ek else 0])

def wait(sec):
    end = time.time() + sec
    while time.time() < end:
        poll(); time.sleep(0.05)

def mode(name):
    mid = m.mode_mapping()[name]
    m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mid)
    wait(1)

def goto(x, y, z_ned, tol=1.0, timeout=40):
    state["sp"] = (x, y, z_ned)
    t_end = time.time() + timeout
    while time.time() < t_end:
        m.mav.set_position_target_local_ned_send(
            0, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000, x, y, z_ned, 0, 0, 0, 0, 0, 0, 0, 0)
        poll()
        lp = last.get("LOCAL_POSITION_NED")
        if lp and math.dist((lp.x, lp.y, lp.z), (x, y, z_ned)) < tol:
            return True
        time.sleep(0.1)
    events.append((time.time() - state["t0"], f"goto timeout {x},{y},{z_ned}"))
    return False

# --- ucus ---
mode("GUIDED")
state["phase"] = "arm"
m.arducopter_arm(); m.motors_armed_wait()
state["phase"] = "takeoff"
m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                        0, 0, 0, 0, 0, 0, 0, args.alt)
state["sp"] = (0, 0, -args.alt)
t_end = time.time() + 40
while time.time() < t_end:
    poll()
    lp = last.get("LOCAL_POSITION_NED")
    if lp and -lp.z > args.alt * 0.95:
        break
    time.sleep(0.1)

state["phase"] = "hover"
goto(0, 0, -args.alt, tol=0.5)
wait(args.hover_s)

state["phase"] = "route"
s = args.side
for (x, y) in [(s, 0), (s, s), (0, s), (0, 0)]:
    goto(x, y, -args.alt, tol=1.0)
    wait(2)

state["phase"] = "land"
mode("LAND")
while True:
    poll()
    lp = last.get("LOCAL_POSITION_NED")
    hb = last.get("HEARTBEAT")
    if (hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)) or any("Disarming" in txt for _, txt in events):
        wait(1); break
    if lp and -lp.z < 0.05 and abs(lp.vz) < 0.05:
        wait(3); break
    if time.time() - state["t0"] > 300:
        break
    time.sleep(0.05)

with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "phase", "x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw",
                "sp_x", "sp_y", "sp_z", "pwm1", "pwm2", "pwm3", "pwm4",
                "vibe_x", "vibe_y", "vibe_z", "clip0", "ekf_flags"])
    w.writerows(rows)
with open(out.replace(".csv", "_events.txt"), "w") as f:
    for t, txt in events:
        f.write(f"{t:.1f}\t{txt}\n")
print("Kaydedildi:", out, len(rows), "satir")
