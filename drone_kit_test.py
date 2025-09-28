# rpi_sitl_client.py
from dronekit import connect, VehicleMode
print("Connecting to SITL from Mac on udp:0.0.0.0:14550 ...")
v = connect('udp:0.0.0.0:14550', wait_ready=True, timeout=60)
print("Mode:", v.mode.name, "| Armable:", v.is_armable)
v.close()
