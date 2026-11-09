#!/usr/bin/env python3
"""Gazebo iris_agricultural modelini bos (12 kg) veya dolu (28 kg + CG kaymasi) moda gecirir."""
import re, shutil, sys

if len(sys.argv) < 2 or sys.argv[1] not in ("empty", "full"):
    print("Kullanim: python3 set_tank_mode.py [empty|full]")
    sys.exit(1)

mode = sys.argv[1]
sdf_path = "/home/beg/spray_drone_sitl/models/iris_agricultural/model.sdf"
txt = open(sdf_path).read()

if mode == "empty":
    new_block = """    <link name='spray_tank_link'>
      <pose relative_to='iris_with_standoffs::base_link'>0 0 -0.10 0 0 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>1e-05</ixx>
          <iyy>1e-05</iyy>
          <izz>1e-05</izz>
        </inertia>
      </inertial>"""
    msg = "[MOD] BOS TANK (12 kg, simetrik CG) Gazebo modeline yazildi."
else:
    new_block = """    <link name='spray_tank_link'>
      <pose relative_to='iris_with_standoffs::base_link'>0.025 0.015 -0.14 0 0 0</pose>
      <inertial>
        <mass>0.85</mass>
        <inertia>
          <ixx>0.012</ixx>
          <iyy>0.012</iyy>
          <izz>0.015</izz>
        </inertia>
      </inertial>"""
    msg = "[MOD] DOLU TANK (28 kg, +2.5cm X / +1.5cm Y / -14cm Z CG kaymasi) Gazebo modeline yazildi."

txt = re.sub(
    r"    <link name='spray_tank_link'>.*?</inertial>",
    new_block,
    txt,
    flags=re.DOTALL,
)
open(sdf_path, "w").write(txt)
shutil.copy(sdf_path, "/home/beg/ardupilot_gazebo/models/iris_agricultural/model.sdf")
shutil.copy("/home/beg/spray_drone_sitl/worlds/farm_field.sdf", "/home/beg/ardupilot_gazebo/worlds/farm_field.sdf")
print(msg)

