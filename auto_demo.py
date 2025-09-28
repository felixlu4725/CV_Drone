# auto_demo.py (on the Pi)
from dronekit import connect, VehicleMode
from autonomous_telemetry import arm_and_takeoff, send_body_velocity

v = connect('udp:0.0.0.0:14550', wait_ready=True, timeout=60)

# take off to 10m
arm_and_takeoff(v, 10)

# fly forward 5 s at 1 m/s (body frame: +X = forward)
send_body_velocity(v, 1.0, 0.0, 0.0, duration_s=5)

# land
v.mode = VehicleMode("LAND")
v.close()
