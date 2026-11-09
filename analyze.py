#!/usr/bin/env python3
"""
Kullanim: python3 analyze.py log_empty.csv log_full.csv
Metrikler: yukselme asimi, hover sapmasi, rota izleme hatasi, roll/pitch salinimi,
motor PWM ortalama/maks/dogruluk, inis dikey hizi, titresim, clipping, olay sayisi.
"""
import sys, json, os
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def metrics(path):
    d = pd.read_csv(path)
    meta_p = path.replace(".csv", "_meta.json")
    meta = json.load(open(meta_p)) if os.path.exists(meta_p) else {}
    alt = -d.z; sp_alt = -d.sp_z
    r = {"label": meta.get("label", path), "mass_kg": meta.get("mass_kg")}

    tk = d[d.phase == "takeoff"]
    r["takeoff_overshoot_m"] = float(alt[tk.index].max() - sp_alt[tk.index].iloc[-1]) if len(tk) else np.nan

    hv = d[d.phase == "hover"]
    r["hover_alt_std_m"] = float((-hv.z).std()) if len(hv) else np.nan
    r["hover_roll_pitch_std_deg"] = float(np.hypot(hv.roll.std(), hv.pitch.std())) if len(hv) else np.nan
    r["hover_xy_std_m"] = float(np.hypot(hv.x.std(), hv.y.std())) if len(hv) else np.nan

    rt = d[d.phase == "route"]
    if len(rt):
        err = np.hypot(rt.x - rt.sp_x, rt.y - rt.sp_y)
        r["route_xy_err_rms_m"] = float(np.sqrt((err ** 2).mean()))
        r["route_alt_err_max_m"] = float((rt.z - rt.sp_z).abs().max())
        r["route_max_tilt_deg"] = float(np.hypot(rt.roll, rt.pitch).max())

    pw = d[["pwm1", "pwm2", "pwm3", "pwm4"]]
    act = d[d.phase.isin(["hover", "route"])][["pwm1", "pwm2", "pwm3", "pwm4"]]
    if len(act):
        r["pwm_mean_hover_route"] = float(act.values.mean())
        r["pwm_max"] = int(pw.values.max())
        r["pwm_imbalance_pct"] = float((act.mean().max() - act.mean().min()) / max(act.values.mean() - 1000, 1) * 100)
        r["sat_pct_time_pwm>1900"] = float((act.values.max(axis=1) > 1900).mean() * 100)

    ld = d[(d.phase == "land") & (alt < 2.0)]
    r["landing_vz_mean_last2m_mps"] = float(ld.vz.mean()) if len(ld) else np.nan
    r["landing_vz_max_mps"] = float(ld.vz.max()) if len(ld) else np.nan   # NED: asagi = +
    r["vibe_max_xyz"] = float(d[["vibe_x", "vibe_y", "vibe_z"]].values.max())
    r["clipping_count"] = int(d.clip0.max())
    ev = path.replace(".csv", "_events.txt")
    r["fc_events"] = sum(1 for _ in open(ev)) if os.path.exists(ev) else None
    return d, r

ds, rs = [], []
for p in sys.argv[1:]:
    d, r = metrics(p); ds.append(d); rs.append(r)
tab = pd.DataFrame(rs).set_index("label").T
print(tab.to_string())
tab.to_csv("metrics_summary.csv")

fig, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=False)
for d, r in zip(ds, rs):
    l = r["label"]
    ax[0].plot(d.t, -d.z, label=l)
    ax[1].plot(d.x, d.y, label=l)
    ax[2].plot(d.t, d.roll, label=f"{l} roll"); ax[2].plot(d.t, d.pitch, "--", label=f"{l} pitch")
    ax[3].plot(d.t, d[["pwm1", "pwm2", "pwm3", "pwm4"]].mean(axis=1), label=f"{l} ort. PWM")
ax[0].set_ylabel("irtifa [m]"); ax[1].set_ylabel("y [m]"); ax[1].set_xlabel("x [m]"); ax[1].axis("equal")
ax[2].set_ylabel("aci [deg]"); ax[3].set_ylabel("PWM [us]"); ax[3].set_xlabel("t [s]")
for a in ax: a.grid(True); a.legend()
plt.tight_layout(); plt.savefig("comparison.png", dpi=130)
print("Yazildi: metrics_summary.csv, comparison.png")
