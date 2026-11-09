# Agricultural Spray Drone – ArduPilot SITL & Gazebo Control Study

[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04.4_LTS-orange.svg)](https://ubuntu.com/)
[![ArduCopter](https://img.shields.io/badge/ArduCopter-V4.8.0--dev-blue.svg)](https://ardupilot.org/)
[![Gazebo Sim](https://img.shields.io/badge/Gazebo_Sim-8.11.0-informational.svg)](https://gazebosim.org/)
[![Python 3.12](https://img.shields.io/badge/Python-3.12.3-green.svg)](https://www.python.org/)
[![Mission Planner](https://img.shields.io/badge/Mission_Planner-1.3.83-yellow.svg)](https://ardupilot.org/planner/)

**Takeoff, hover, route tracking, and landing control analysis for a 16 L agricultural spraying multicopter under empty vs. full tank mass and CG shifts using ArduPilot SITL, Gazebo Sim, and Mission Planner.**

> **Note**: All vehicle mass, inertia, thrust scaling, and tank geometry figures in this repository are labeled **assumptions** used for simulation analysis, not proprietary product specifications.

---

## System in Action

<div align="center">
  <table>
    <tr>
      <td width="50%">
        <img src="figures/demo_field_mission.gif" alt="16L Spray Drone Takeoff & Agricultural Mission" width="100%"/>
        <br/>
        <em>16 L agricultural drone on precision landing pad with Mission Planner 43-waypoint survey grid</em>
      </td>
      <td width="50%">
        <img src="figures/demo_sitl_scenarios.gif" alt="Automated Comparative Scenario Execution" width="100%"/>
        <br/>
        <em>Automated MAVLink scenario runner (empty vs. full vs. full_tuned) and precision pad landing</em>
      </td>
    </tr>
    <tr>
      <td width="50%">
        <img src="figures/gazebo_farm_field.jpg" alt="Gazebo Agricultural Field & Mission Planner" width="100%"/>
        <br/>
        <em>Custom Gazebo agricultural field aligned with Mission Planner survey grid and MAVProxy console</em>
      </td>
      <td width="50%">
        <img src="figures/spray_tank_landing_pad.jpg" alt="16L Spray Tank & Landing Pad" width="100%"/>
        <br/>
        <em>Modified Iris model with 16 L cylindrical tank, spray boom/nozzles, and precision landing pad</em>
      </td>
    </tr>
  </table>
</div>

## Telemetry Comparison (Empty vs. Full vs. Full Tuned)

<div align="center">
  <img src="figures/comparison.png" alt="Empty vs Full vs Full Tuned Comparison" width="850"/>
  <br/>
  <em>Comparison of altitude hold, 20×20 m route tracking, roll/pitch response, and mean motor PWM across three scenarios</em>
</div>

---

## Project Overview

Agricultural spraying drones experience a large mass and inertia shift between takeoff (full 16 L tank) and landing (empty tank). Because `MOT_THST_HOVER` in ArduPilot is a controller parameter rather than a direct physical measurement, changing vehicle mass in the physics engine without updating hover throttle, acceleration limits, and rate gains degrades altitude hold, increases corner overshoot, and narrows the actuator saturation margin.

This repository provides a reproducible testbench that:
- Models **empty (12 kg)** and **full (28 kg, +CG offset)** configurations in Gazebo (`models/iris_agricultural/model.sdf`) inside a custom farm environment (`worlds/farm_field.sdf`).
- Executes automated **takeoff → 15 s hover → 20×20 m square route → LAND** flights via PyMAVLink (`run_scenario.py`) while logging 20 Hz telemetry, motor PWM outputs, vibration, and flight controller events.
- Computes comparative flight control metrics and plots (`analyze.py` → `metrics_summary.csv`, `figures/comparison.png`).
- Documents a safe stage-gate field commissioning and rollback plan (`report_draft.md` and `report/DRONEQUBE_Teknik_Rapor_Begum_Aydogan_Cicin.pdf`).

---

## System Architecture

```
┌────────────────────┐      JSON / Lockstep      ┌──────────────────────┐
│  ArduCopter SITL   │◄─────────────────────────►│     Gazebo Sim       │
│  (V4.8.0-dev)      │       (UDP Port 9002)     │ (iris_agricultural + │
└─────────┬──────────┘                           │   farm_field.sdf)    │
          │                                      └──────────────────────┘
          │ TCP 5760
          ▼
┌────────────────────┐
│     MAVProxy       │
│ (Console + Map)    │
└────┬──────────┬────┘
     │          │
     │ UDP      │ UDP 14560
     │ 14550    ▼
     │    ┌────────────────────┐     ┌───────────────────────┐
     │    │  run_scenario.py   │────►│      analyze.py       │
     │    │ (20 Hz Telemetry)  │     │ (CSV Metrics + Plots) │
     │    └────────────────────┘     └───────────────────────┘
     ▼
┌────────────────────┐
│  Mission Planner   │
│ (Survey Grid / HUD)│
└────────────────────┘
```

---

## Environment & Versions

| Component | Version |
|---|---|
| **OS** | Ubuntu 24.04.4 LTS (x86_64) |
| **Autopilot Firmware** | ArduCopter V4.8.0-dev (`b0af1735`) |
| **3D Physics Simulator** | Gazebo Sim `8.11.0` + `ardupilot_gazebo` plugin |
| **Ground Control Station** | Mission Planner `1.3.83` (via Mono) + MAVProxy |
| **Python / PyMAVLink** | Python `3.12.3`, `pymavlink 2.4.50` |

---

## Running the System

You need **3 separate terminals** to run the simulation and automated test suite.

### Prerequisites

```bash
# Activate ArduPilot virtual environment and install analysis dependencies
source ~/venv-ardupilot/bin/activate
pip install -r requirements.txt

# Copy custom model and world to ardupilot_gazebo
cp -r models/iris_agricultural ~/ardupilot_gazebo/models/
cp worlds/farm_field.sdf ~/ardupilot_gazebo/worlds/
```

### Terminal 1: Launch Gazebo Farm World

```bash
gz sim -v4 -r farm_field.sdf
```

### Terminal 2: Launch ArduCopter SITL & Open Extra Telemetry Port

```bash
source ~/venv-ardupilot/bin/activate
cd ~/ardupilot/Tools/autotest
./sim_vehicle.py -v ArduCopter -f gazebo-iris --model json --console --map -w
```

Once `GPS: OK6` and `EKF3 IMU0 origin set` appear in the MAVProxy console, add port `14560` so Mission Planner (`14550`) and `run_scenario.py` (`14560`) do not conflict:

```text
output add 127.0.0.1:14560
param save params/backup_original.parm
```

### Terminal 3: Run Scenarios & Generate Analysis

```bash
source ~/venv-ardupilot/bin/activate
cd ~/spray_drone_sitl

# Scenario A – Empty baseline (12 kg assumption, centered CG)
python3 run_scenario.py --label empty --mass 12 --set MOT_HOVER_LEARN=0

# Scenario B – Full tank untuned (after updating mass, inertia, and CG in model.sdf & restarting Gazebo/SITL)
python3 run_scenario.py --label full --mass 28 --set MOT_HOVER_LEARN=0

# Scenario C – Full tank tuned (updated hover throttle and acceleration limits)
python3 run_scenario.py --label full_tuned --mass 28 \
  --set MOT_HOVER_LEARN=0 MOT_THST_HOVER=0.68 WPNAV_ACCEL=150 ATC_ACC_R_MAX=360 ATC_ACC_P_MAX=360 LAND_SPEED=40

# Move logs and compute comparative metrics + plot
mv log_*.csv log_*_meta.json log_*_events.txt logs/
python3 analyze.py logs/log_empty.csv logs/log_full.csv logs/log_full_tuned.csv
mv comparison.png figures/
```

---

## Experimental Results Summary

Metrics extracted by `analyze.py` (`metrics_summary.csv`) from the three SITL + Gazebo flights:

| Metric | `empty` (12 kg) | `full` (28 kg, untuned) | `full_tuned` (28 kg, tuned) | Engineering Observation |
|---|---:|---:|---:|---|
| **`pwm_mean_hover_route` (µs)** | `1561.0` | `1633.9` | `1634.8` | Mean motor output rises by **+73 µs** under full load |
| **`pwm_max` (µs)** | `1665` | `1950` | `1950` | Heavy turns hit motor ceiling (`1950 µs`) in full load |
| **`sat_pct_time_pwm>1900` (%)** | `0.00%` | `2.48%` | `3.07%` | ~2.5–3% time spent near saturation; requires lower accel limits |
| **`pwm_imbalance_pct` (%)** | `0.52%` | `7.62%` | `7.85%` | CG offset (`+2.5 cm X, +1.5 cm Y, -14 cm Z`) causes **7.7% motor imbalance** |
| **`hover_roll_pitch_std_deg` (°)** | `0.22°` | `6.17°` | `5.08°` | Higher inertia increases oscillation; tuning reduces it by **18%**, full fix needs Rate PID AutoTune |
| **`hover_xy_std_m` (m)** | `0.00 m` | `0.35 m` | `0.21 m` | Position wander reduced by **40%** after hover/accel tuning |
| **`route_alt_err_max_m` (m)** | `0.04 m` | `0.41 m` | `0.31 m` | Corner altitude droop reduced from **41 cm to 31 cm** |
| **`landing_vz_mean_last2m_mps` (m/s)** | `0.29 m/s` | `0.27 m/s` | `0.27 m/s` | Controlled final descent (`max 0.50 m/s`, matching `LAND_SPEED`) |

---

## Project Structure

```
spray_drone_sitl/
├── run_scenario.py                 # Automated MAVLink flight runner & 20 Hz logger
├── analyze.py                      # Metric extractor and 4-panel plot generator
├── requirements.txt                # Python package dependencies
├── metrics_summary.csv             # Summary table of all 16 flight metrics
├── report_draft.md                 # Technical report markdown source
├── report/
│   └── DRONEQUBE_Teknik_Rapor_Begum_Aydogan_Cicin.pdf  # 5-page technical report (PDF)
├── models/
│   └── iris_agricultural/          # Custom Gazebo drone model with 16 L tank & spray boom
├── worlds/
│   └── farm_field.sdf              # Custom Gazebo agricultural field + landing pad world
├── missions/
│   └── tarla_mission.txt           # Mission Planner survey grid waypoint file
├── params/
│   ├── backup_original.parm        # Original ArduCopter parameter backup
│   ├── before_empty.parm           # Baseline empty tank parameter set
│   └── before_full_tuned.parm      # Tuned full tank parameter set
├── logs/
│   ├── log_empty.csv               # Scenario A telemetry, metadata, and FC events
│   ├── log_full.csv                # Scenario B telemetry, metadata, and FC events
│   ├── log_full_tuned.csv          # Scenario C telemetry, metadata, and FC events
│   └── failed/
│       └── failed_trials_notes.txt # Notes on failed thrust/inertia scaling trials
└── figures/
    ├── comparison.png              # 4-panel comparative telemetry plot
    ├── gazebo_farm_field.jpg       # Gazebo + Mission Planner + MAVProxy screenshot
    └── spray_tank_landing_pad.jpg  # 16 L tank and landing pad close-up
```

---

## Simulator Limitations & Field Notes

1. **Liquid Sloshing**: Gazebo models the tank as a rigid body (`<mass>` + `<inertia>` + `<pose>` offset). Free-surface liquid sloshing during half-tank turns must be mitigated mechanically with internal baffles and verified in tethered/gradual-load field tests.
2. **Ground Effect & Wind**: Aerodynamic ground cushioning in the final 2 m and prop-wash turbulence are simplified in Gazebo.
3. **Vibration & Notch Filtering**: Simulated IMU vibration (`vibe_max_xyz ≈ 12–15 m/s²`) does not capture real structural resonances. On hardware, a tethered hover log with `INS_LOG_BAT_MASK=1` is required to configure `INS_HNTCH_*` (throttle or ESC-RPM tracked) prior to running `QuikTune` / `AutoTune`.
4. **Safety Interlocks**: Battery failsafes (`BATT_FS_*`), RC failsafes (`FS_THR_ENABLE`), geofence (`FENCE_ENABLE`), and pre-arm checks (`ARMING_CHECK`) are kept enabled at all times.
