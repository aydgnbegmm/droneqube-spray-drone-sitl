#!/usr/bin/env python3
"""
Mission-driven Gazebo (gz-sim, Harmonic) world generator for an agricultural
spraying drone SITL setup (ArduPilot + ardupilot_gazebo).

Reads a QGC WPL 110 mission file, converts every waypoint to local ENU metres
around the mission HOME (WP0), fits a rotated rectangle to the spray waypoints
and builds the whole farm scene around it, so the field in Gazebo lines up
with the mission by construction.

Gazebo world frame (default): X = East, Y = North, Z = Up  (ENU).
Use --frame nwu only if your ArduPilot plugin uses a custom NWU transform.

Usage:
    python3 generate_world.py tarla_mission.txt -o farm_field.sdf
"""
import argparse
import math
import random
import sys

WGS84_A = 6378137.0
WGS84_E2 = 0.00669437999014


# --------------------------------------------------------------------------
# Geodesy / mission parsing
# --------------------------------------------------------------------------
def geodetic_to_enu(home_lat, home_lon, lat, lon):
    phi = math.radians(home_lat)
    s = math.sin(phi)
    w = math.sqrt(1.0 - WGS84_E2 * s * s)
    r_n = WGS84_A / w
    r_m = WGS84_A * (1.0 - WGS84_E2) / w ** 3
    north = math.radians(lat - home_lat) * r_m
    east = math.radians(lon - home_lon) * r_n * math.cos(phi)
    return east, north


def read_mission(path):
    home = None
    takeoff_alt = 5.0
    wps = []
    with open(path) as fh:
        for line in fh:
            parts = line.split()
            if not parts or parts[0].startswith("QGC"):
                continue
            seq, _cur, _frame, cmd = (int(float(x)) for x in parts[:4])
            vals = [float(x) for x in parts[4:11]]
            lat, lon, alt = vals[4], vals[5], vals[6]
            if cmd == 16 and seq == 0:
                home = (lat, lon, alt)
            elif cmd == 16:
                wps.append({"seq": seq, "lat": lat, "lon": lon, "alt": alt})
            elif cmd == 22:
                takeoff_alt = alt
    if home is None:
        sys.exit("Mission has no HOME (seq 0) item")
    for w in wps:
        w["e"], w["n"] = geodetic_to_enu(home[0], home[1], w["lat"], w["lon"])
    return home, takeoff_alt, wps


def fit_field(pts, margin):
    """Oriented rectangle: local +y = long axis (pointing north-ish), +x across."""
    n = len(pts)
    cx = sum(p[0] for p in pts) / n
    cy = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - cx) ** 2 for p in pts)
    syy = sum((p[1] - cy) ** 2 for p in pts)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in pts)
    ang = 0.5 * math.atan2(2 * sxy, sxx - syy)
    ue, un = math.cos(ang), math.sin(ang)
    if un < 0:
        ue, un = -ue, -un
    ve, vn = un, -ue
    a = [(p[0] - cx) * ve + (p[1] - cy) * vn for p in pts]
    b = [(p[0] - cx) * ue + (p[1] - cy) * un for p in pts]
    ac, bc = (max(a) + min(a)) / 2, (max(b) + min(b)) / 2
    return {
        "cx": cx + ac * ve + bc * ue,
        "cy": cy + ac * vn + bc * un,
        "W": max(a) - min(a) + 2 * margin,
        "L": max(b) - min(b) + 2 * margin,
        "yaw": math.atan2(-ue, un),
        "u": (ue, un),
        "v": (ve, vn),
    }


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    d2 = dx * dx + dy * dy
    t = 0.0 if d2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / d2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


# --------------------------------------------------------------------------
# SDF helpers
# --------------------------------------------------------------------------
def pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return f"{x:.4f} {y:.4f} {z:.4f} {roll:.5f} {pitch:.5f} {yaw:.5f}"


def ptag(*a, **k):
    return f"<pose>{pose(*a, **k)}</pose>"


def g_box(x, y, z):
    return f"<geometry><box><size>{x:.4f} {y:.4f} {z:.4f}</size></box></geometry>"


def g_cyl(r, l):
    return f"<geometry><cylinder><radius>{r:.4f}</radius><length>{l:.4f}</length></cylinder></geometry>"


def g_sph(r):
    return f"<geometry><sphere><radius>{r:.4f}</radius></sphere></geometry>"


def material(c):
    r, g, b = c[:3]
    a = c[3] if len(c) > 3 else 1.0
    return (f"<material><ambient>{r:.3f} {g:.3f} {b:.3f} {a:.2f}</ambient>"
            f"<diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.2f}</diffuse>"
            f"<specular>0.04 0.04 0.04 1</specular></material>")


def vis(name, geom, color, pz=None, transp=0.0, shadow=True):
    t = f"<transparency>{transp:.2f}</transparency>" if transp else ""
    s = "" if shadow else "<cast_shadows>false</cast_shadows>"
    return f"<visual name='{name}'>{s}{pz or ''}{geom}{material(color)}{t}</visual>"


def col(name, geom, pz=None):
    return f"<collision name='{name}'>{pz or ''}{geom}</collision>"


class Scene:
    def __init__(self):
        self.links = []
        self.names = set()

    def link(self, name, x, y, z, yaw, *parts):
        assert name not in self.names, name
        self.names.add(name)
        self.links.append(
            f"      <link name='{name}'>{ptag(x, y, z, yaw=yaw)}{''.join(parts)}</link>")


# --------------------------------------------------------------------------
# Scene builders
# --------------------------------------------------------------------------
def build_field(sc, fld, args, rng):
    W, L = fld["W"], fld["L"]
    parts = []
    parts.append(vis("headland", g_box(W + 5, L + 5, 0.02), (0.50, 0.42, 0.30), ptag(0, 0, 0.01), shadow=False))
    parts.append(vis("soil", g_box(W, L, 0.012), (0.36, 0.24, 0.14), ptag(0, 0, 0.024), shadow=False))

    pitch = args.row_pitch
    x = -W / 2 + 1.5
    i = 0
    while x <= W / 2 - 1.5:
        g = rng.uniform(0.52, 0.70)
        parts.append(vis(f"ridge_{i}", g_box(0.70, L - 2.0, 0.08), (0.30, 0.20, 0.11), ptag(x, 0, 0.07), shadow=False))
        parts.append(vis(f"crop_{i}", g_box(0.42, L - 3.0 - rng.uniform(0, 1.5), 0.28),
                         (0.14 + rng.uniform(0, 0.05), g, 0.12 + rng.uniform(0, 0.04)), ptag(x, 0, 0.25)))
        x += pitch
        i += 1

    # fence
    off = 4.5
    hx, hy = W / 2 + off, L / 2 + off
    for k, (z, tag) in enumerate(((0.55, "lo"), (1.05, "hi"))):
        for sx in (-1, 1):
            parts.append(vis(f"rail_y_{tag}_{sx}", g_box(0.03, 2 * hy, 0.03), (0.55, 0.45, 0.30), ptag(sx * hx, 0, z), shadow=False))
        for sy in (-1, 1):
            parts.append(vis(f"rail_x_{tag}_{sy}", g_box(2 * hx, 0.03, 0.03), (0.55, 0.45, 0.30), ptag(0, sy * hy, z), shadow=False))
    pid = 0
    for sx in (-1, 1):
        n = math.ceil(2 * hy / 6.0)
        for j in range(n + 1):
            parts.append(vis(f"post_{pid}", g_cyl(0.06, 1.3), (0.40, 0.30, 0.20), ptag(sx * hx, -hy + j * 2 * hy / n, 0.65)))
            pid += 1
    for sy in (-1, 1):
        n = math.ceil(2 * hx / 6.0)
        for j in range(1, n):
            parts.append(vis(f"post_{pid}", g_cyl(0.06, 1.3), (0.40, 0.30, 0.20), ptag(-hx + j * 2 * hx / n, sy * hy, 0.65)))
            pid += 1

    # survey stakes at the 4 corners of the planted soil
    for cxs in (-1, 1):
        for cys in (-1, 1):
            parts.append(vis(f"stake_{cxs}_{cys}", g_cyl(0.07, 1.8), (0.95, 0.35, 0.05), ptag(cxs * W / 2, cys * L / 2, 0.9)))
    sc.link("field", fld["cx"], fld["cy"], 0.0, fld["yaw"], *parts)


def build_pad(sc):
    p = [
        vis("base", g_box(4.0, 4.0, 0.01), (0.62, 0.62, 0.60), ptag(0, 0, 0.005), shadow=False),
        vis("ring", g_cyl(1.45, 0.002), (0.95, 0.78, 0.05), ptag(0, 0, 0.011), shadow=False),
        vis("inner", g_cyl(1.2, 0.002), (0.62, 0.62, 0.60), ptag(0, 0, 0.012), shadow=False),
        vis("h_left", g_box(0.2, 1.4, 0.002), (0.05, 0.05, 0.05), ptag(-0.4, 0, 0.0135), shadow=False),
        vis("h_right", g_box(0.2, 1.4, 0.002), (0.05, 0.05, 0.05), ptag(0.4, 0, 0.0135), shadow=False),
        vis("h_bar", g_box(0.6, 0.2, 0.002), (0.05, 0.05, 0.05), ptag(0, 0, 0.0135), shadow=False),
    ]
    sc.link("landing_pad", 0, 0, 0, 0, *p)


def build_track(sc, fld, home_xy):
    cx, cy, W, L = fld["cx"], fld["cy"], fld["W"], fld["L"]
    ve, vn = fld["v"]
    ue, un = fld["u"]
    best = None
    for sx in (-1, 1):
        for sy in (-1, 1):
            x = cx + sx * (W / 2 + 2.5) * ve + sy * (L / 2 + 2.5) * ue
            y = cy + sx * (W / 2 + 2.5) * vn + sy * (L / 2 + 2.5) * un
            d = math.hypot(x - home_xy[0], y - home_xy[1])
            if best is None or d < best[0]:
                best = (d, x, y)
    d, x, y = best
    yaw = math.atan2(y - home_xy[1], x - home_xy[0])
    sc.link("farm_track", (x + home_xy[0]) / 2, (y + home_xy[1]) / 2, 0, yaw,
            vis("track", g_box(d, 3.0, 0.004), (0.55, 0.48, 0.38), ptag(0, 0, 0.007), shadow=False))


def build_patches(sc, rng, count=60):
    placed = []
    tries = 0
    i = 0
    greens = [(0.30, 0.52, 0.22), (0.26, 0.46, 0.20), (0.36, 0.56, 0.24), (0.42, 0.55, 0.26), (0.33, 0.48, 0.19)]
    while len(placed) < count and tries < 4000:
        tries += 1
        x, y = rng.uniform(-300, 250), rng.uniform(-250, 260)
        a, b = rng.uniform(8, 26), rng.uniform(8, 26)
        r = math.hypot(a, b) / 2
        if any(math.hypot(x - px, y - py) < r + pr + 0.5 for px, py, pr in placed):
            continue
        placed.append((x, y, r))
        sc.link(f"patch_{i}", x, y, 0, rng.uniform(0, math.pi),
                vis("v", g_box(a, b, 0.004), rng.choice(greens), ptag(0, 0, 0.002), shadow=False))
        i += 1


def build_barn(sc, x, y, yaw):
    wall = (0.55, 0.14, 0.11)
    roof = (0.28, 0.28, 0.30)
    ang = math.atan2(2.5, 4.0)
    span = math.hypot(4.0, 2.5) + 0.3
    parts = [
        vis("walls", g_box(12, 8, 4), wall, ptag(0, 0, 2)),
        col("walls_c", g_box(12, 8, 4), ptag(0, 0, 2)),
        vis("roof_p", g_box(12.6, span, 0.14), roof, ptag(0, 2.0, 5.25, roll=-ang)),
        col("roof_p_c", g_box(12.6, span, 0.14), ptag(0, 2.0, 5.25, roll=-ang)),
        vis("roof_n", g_box(12.6, span, 0.14), roof, ptag(0, -2.0, 5.25, roll=ang)),
        col("roof_n_c", g_box(12.6, span, 0.14), ptag(0, -2.0, 5.25, roll=ang)),
        vis("door", g_box(3.6, 0.1, 3.0), (0.20, 0.12, 0.08), ptag(0, -4.05, 1.5)),
    ]
    sc.link("barn", x, y, 0, yaw, *parts)


def build_yard(sc):
    sc.link("container", 14, -9, 0, 0.3,
            vis("v", g_box(6, 2.4, 2.6), (0.10, 0.30, 0.55), ptag(0, 0, 1.3)),
            col("c", g_box(6, 2.4, 2.6), ptag(0, 0, 1.3)))
    sc.link("water_tank", 9, -5, 0, 0,
            vis("v", g_cyl(0.9, 1.8), (0.85, 0.88, 0.90), ptag(0, 0, 0.9)),
            col("c", g_cyl(0.9, 1.8), ptag(0, 0, 0.9)),
            vis("band", g_cyl(0.92, 0.15), (0.10, 0.35, 0.75), ptag(0, 0, 1.2)))
    sc.link("pallet", 9, -7.2, 0, 0.2,
            vis("v", g_box(1.2, 0.8, 0.15), (0.55, 0.40, 0.22), ptag(0, 0, 0.075)))
    # wind sock
    parts = [vis("pole", g_cyl(0.04, 4.0), (0.8, 0.8, 0.8), ptag(0, 0, 2.0)),
             col("pole_c", g_cyl(0.04, 4.0), ptag(0, 0, 2.0))]
    rings = [(0.30, (1.0, 0.45, 0.05)), (0.26, (1.0, 1.0, 1.0)), (0.22, (1.0, 0.45, 0.05)), (0.18, (1.0, 1.0, 1.0))]
    for k, (r, c) in enumerate(rings):
        parts.append(vis(f"sock_{k}", g_cyl(r, 0.3), c, ptag(0.15 + 0.3 * k + 0.3, 0, 3.9, pitch=math.pi / 2)))
    sc.link("wind_sock", 8, 9, 0, 0, *parts)


def build_trees(sc, fld, route, keepouts, rng, home_xy):
    cx, cy, W, L = fld["cx"], fld["cy"], fld["W"], fld["L"]
    ve, vn = fld["v"]
    ue, un = fld["u"]
    placed = []

    def clear(x, y, extra):
        lx = (x - cx) * ve + (y - cy) * vn
        ly = (x - cx) * ue + (y - cy) * un
        if abs(lx) < W / 2 + 18 and abs(ly) < L / 2 + 18:
            return False
        if math.hypot(x - home_xy[0], y - home_xy[1]) < 20:
            return False
        for (ax, ay), (bx, by) in zip(route, route[1:]):
            if seg_dist(x, y, ax, ay, bx, by) < 14 + extra:
                return False
        if any(math.hypot(x - kx, y - ky) < kr + extra for kx, ky, kr in keepouts):
            return False
        return all(math.hypot(x - px, y - py) >= 7 for px, py in placed)

    def tree(i, x, y, poplar):
        if poplar:
            h = rng.uniform(8, 11)
            parts = [vis("trunk", g_cyl(0.2, 3.0), (0.35, 0.25, 0.15), ptag(0, 0, 1.5)),
                     col("trunk_c", g_cyl(0.2, 3.0), ptag(0, 0, 1.5)),
                     vis("crown", g_cyl(1.3, h), (0.12, 0.38 + rng.uniform(0, 0.1), 0.14), ptag(0, 0, 3.0 + h / 2)),
                     col("crown_c", g_cyl(1.3, h), ptag(0, 0, 3.0 + h / 2))]
        else:
            r = rng.uniform(2.2, 3.2)
            parts = [vis("trunk", g_cyl(0.25, 3.2), (0.35, 0.25, 0.15), ptag(0, 0, 1.6)),
                     col("trunk_c", g_cyl(0.25, 3.2), ptag(0, 0, 1.6)),
                     vis("crown", g_sph(r), (0.14 + rng.uniform(0, 0.06), 0.40 + rng.uniform(0, 0.12), 0.14), ptag(0, 0, 3.2 + 0.6 * r)),
                     col("crown_c", g_sph(r), ptag(0, 0, 3.2 + 0.6 * r))]
        sc.link(f"tree_{i}", x, y, 0, 0, *parts)

    # windbreak on the side away from home
    side = -1.0 if ((home_xy[0] - cx) * ve + (home_xy[1] - cy) * vn) > 0 else 1.0
    i = 0
    ly = -L / 2
    while ly <= L / 2:
        lx = side * (W / 2 + 16 + rng.uniform(-1, 1))
        x, y = cx + lx * ve + ly * ue, cy + lx * vn + ly * un
        if clear(x, y, 0):
            placed.append((x, y))
            tree(i, x, y, True)
            i += 1
        ly += 8
    tries = 0
    while i < 36 and tries < 5000:
        tries += 1
        x, y = cx + rng.uniform(-190, 190), cy + rng.uniform(-190, 190)
        if clear(x, y, 0):
            placed.append((x, y))
            tree(i, x, y, False)
            i += 1
    return i


def build_route(sc, home_xy, takeoff_alt, wps):
    pts = [(home_xy[0], home_xy[1], takeoff_alt)] + [(w["e"], w["n"], w["alt"]) for w in wps]
    for k, (a, b) in enumerate(zip(pts, pts[1:])):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d < 0.1:
            continue
        sc.link(f"route_{k}", (a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2,
                math.atan2(b[1] - a[1], b[0] - a[0]),
                vis("v", g_box(d, 0.06, 0.06), (1.0, 0.85, 0.1), transp=0.45, shadow=False))
    sc.link("takeoff_marker", home_xy[0], home_xy[1], 0, 0,
            vis("pole", g_cyl(0.03, takeoff_alt), (0.1, 0.9, 0.2), ptag(0, 0, takeoff_alt / 2), transp=0.6, shadow=False))
    for k, w in enumerate(wps):
        c = (0.1, 0.9, 0.2) if k == 0 else ((0.95, 0.15, 0.15) if k == len(wps) - 1 else (0.1, 0.45, 0.95))
        sc.link(f"wp_{w['seq']}", w["e"], w["n"], 0, 0,
                vis("pole", g_cyl(0.03, w["alt"]), c, ptag(0, 0, w["alt"] / 2), transp=0.6, shadow=False),
                vis("top", g_sph(0.28), c, ptag(0, 0, w["alt"]), transp=0.35, shadow=False))


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mission")
    ap.add_argument("-o", "--output", default="farm_field.sdf")
    ap.add_argument("--model", default="model://iris_agricultural")
    ap.add_argument("--margin", type=float, default=3.0, help="field margin around outermost WPs [m]")
    ap.add_argument("--row-pitch", type=float, default=0.75, help="crop row spacing [m]")
    ap.add_argument("--frame", choices=["enu", "nwu"], default="enu")
    ap.add_argument("--no-markers", action="store_true", help="omit WP poles and route ribbon")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    home, takeoff_alt, wps = read_mission(args.mission)
    home_xy = (0.0, 0.0)
    fld = fit_field([(w["e"], w["n"]) for w in wps], args.margin)

    sc = Scene()
    build_patches(sc, rng)
    build_pad(sc)
    build_track(sc, fld, home_xy)
    build_field(sc, fld, args, rng)

    barn = (32.0, 18.0, 0.25)
    build_barn(sc, *barn)
    build_yard(sc)
    keepouts = [(barn[0], barn[1], 9.0), (14, -9, 6.0), (9, -5, 4.0), (8, 9, 4.0)]
    route = [home_xy] + [(w["e"], w["n"]) for w in wps]
    n_trees = build_trees(sc, fld, route, keepouts, rng, home_xy)
    if not args.no_markers:
        build_route(sc, home_xy, takeoff_alt, wps)

    root_yaw = 0.0 if args.frame == "enu" else -math.pi / 2

    sdf = f"""<?xml version="1.0" ?>
<!-- Generated by generate_world.py from {args.mission} -->
<sdf version="1.9">
  <world name="farm_field">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>

    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.53 0.74 0.92 1</background>
      <shadows>true</shadows>
      <sky></sky>
    </scene>

    <!-- World origin == mission HOME (WP0) -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{home[0]:.6f}</latitude_deg>
      <longitude_deg>{home[1]:.6f}</longitude_deg>
      <elevation>{round(home[2])}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>0.9 0.88 0.82 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1200 1200</size></plane></geometry>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>1200 1200</size></plane></geometry>
          <material>
            <ambient>0.28 0.48 0.22 1</ambient>
            <diffuse>0.28 0.48 0.22 1</diffuse>
            <specular>0.02 0.02 0.02 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Farm scene (frame: {args.frame.upper()}). All positions come from the mission. -->
    <model name="farm_scene">
      <static>true</static>
      <pose>0 0 0 0 0 {root_yaw:.6f}</pose>
{chr(10).join(sc.links)}
    </model>

    <include>
      <uri>{args.model}</uri>
      <pose degrees="true">0 0 0.195 0 0 90</pose>
    </include>
  </world>
</sdf>
"""
    with open(args.output, "w") as fh:
        fh.write(sdf)

    # ---- report / self-check
    ue, un = fld["u"]
    ve, vn = fld["v"]
    inside = 0
    for w in wps:
        lx = (w["e"] - fld["cx"]) * ve + (w["n"] - fld["cy"]) * vn
        ly = (w["e"] - fld["cx"]) * ue + (w["n"] - fld["cy"]) * un
        inside += abs(lx) <= fld["W"] / 2 and abs(ly) <= fld["L"] / 2
    print(f"Home            : lat {home[0]:.6f}  lon {home[1]:.6f}  alt {home[2]:.1f}")
    print(f"Waypoints       : {len(wps)}  (inside field rectangle: {inside})")
    print(f"Field center ENU: E {fld['cx']:.2f}  N {fld['cy']:.2f}   -> Gazebo (x,y) = ({fld['cx']:.2f}, {fld['cy']:.2f}) in ENU")
    print(f"Field size      : {fld['W']:.1f} m (across) x {fld['L']:.1f} m (long axis)")
    print(f"Field yaw       : {math.degrees(fld['yaw']):.2f} deg from North axis (CCW +)")
    print(f"Trees           : {n_trees}   Links total: {len(sc.links)}   Frame: {args.frame}")
    print(f"Written         : {args.output}")


if __name__ == "__main__":
    main()
