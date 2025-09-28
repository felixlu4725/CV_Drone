#!/usr/bin/env python3
"""
Autonomous flight telemetry + control scaffold (DroneKit + MAVLink)

- Connects to FC or SITL
- Requests high-rate MAVLink streams
- Reads & prints essential state for autonomous control
- Provides helpers: arm_and_takeoff, goto_latlonalt, send_body_velocity

Run examples:
  # SITL on Mac -> Pi
  python autonomy_telemetry.py --conn udp:0.0.0.0:14550

  # Real FC over USB on Pi
  python autonomy_telemetry.py --conn /dev/ttyUSB0 --baud 115200

  # Real FC over UART (GPIO) on Pi
  python autonomy_telemetry.py --conn /dev/ttyAMA0 --baud 57600
"""

import argparse, time, csv, sys, math, threading
from dataclasses import dataclass, asdict
from typing import Optional

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


@dataclass
class Telemetry:
    stamp: float = 0.0
    mode: str = "UNKNOWN"
    armed: bool = False
    system_status: str = "UNKNOWN"
    is_armable: bool = False

    # GPS / Position
    fix_type: int = 0
    sats: int = 0
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt_msl: Optional[float] = None       # meters AMSL
    alt_rel: Optional[float] = None       # meters AGL (relative)

    # Motion / Attitude
    roll: Optional[float] = None          # rad
    pitch: Optional[float] = None         # rad
    yaw: Optional[float] = None           # rad
    vx: Optional[float] = None            # m/s (NED)
    vy: Optional[float] = None
    vz: Optional[float] = None
    groundspeed: Optional[float] = None   # m/s
    airspeed: Optional[float] = None      # m/s
    heading: Optional[float] = None       # deg (0..360)

    # Sensors / Health
    rngf_dist: Optional[float] = None     # m (rangefinder)
    batt_voltage: Optional[float] = None  # V
    batt_current: Optional[float] = None  # A (may be None)
    batt_level: Optional[float] = None    # % (may be None)
    last_hb: Optional[float] = None       # seconds
    ekf_ok: Optional[bool] = None         # coarse flag (from EKF report if available)

def fmt(v, spec):
    """Format None as 'nan', otherwise use the given format spec."""
    return format(v, spec) if v is not None else "nan"


def request_message_rates(vehicle, hz: int = 10):
    """Ask the FC to send key messages at `hz` using SET_MESSAGE_INTERVAL."""
    MASTER = vehicle._master  # low-level MAVLink connection
    send = MASTER.mav.command_long_send
    target_sys = vehicle._master.target_system
    target_comp = vehicle._master.target_component

    def set_rate(msg_id, rate_hz):
        interval_us = int(1e6 / max(1, rate_hz))
        MASTER.mav.set_message_interval_send(
            target_sys, msg_id, interval_us, 0, 0
        )

    # Common messages you want for autonomy
    msgs_10hz = [
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
    ]
    msgs_2hz = [
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
        mavutil.mavlink.MAVLINK_MSG_ID_RANGEFINDER,
    ]

    for mid in msgs_10hz:
        try: set_rate(mid, hz)
        except Exception: pass
    for mid in msgs_2hz:
        try: set_rate(mid, max(1, hz // 5))
        except Exception: pass


def arm_and_takeoff(vehicle, alt_m=10.0, timeout=60):
    """Arm in GUIDED and take off to alt_m (AGL)."""
    t0 = time.time()
    while not vehicle.is_armable:
        if time.time() - t0 > timeout:
            raise TimeoutError("Not armable (sensors/GPS not ready)")
        time.sleep(0.5)

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        if time.time() - t0 > timeout:
            raise TimeoutError("Mode change to GUIDED timed out")
        time.sleep(0.2)

    vehicle.armed = True
    while not vehicle.armed:
        if time.time() - t0 > timeout:
            raise TimeoutError("Arming timed out")
        time.sleep(0.2)

    vehicle.simple_takeoff(alt_m)
    while True:
        alt = vehicle.location.global_relative_frame.alt or 0.0
        if alt >= alt_m * 0.95:
            break
        if time.time() - t0 > timeout:
            raise TimeoutError("Takeoff altitude not reached")
        time.sleep(0.5)


def goto_latlonalt(vehicle, lat, lon, alt_rel_m):
    """Simple goto in GUIDED (lat/lon WGS84, alt AGL meters)."""
    loc = LocationGlobalRelative(lat, lon, alt_rel_m)
    vehicle.simple_goto(loc)


def send_body_velocity(vehicle, vx, vy, vz, duration_s=0.0):
    """
    Body-frame velocity command (m/s).
    +X forward, +Y right, +Z down.
    If duration_s>0, block and keep sending at ~10Hz.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms
        0, 0,  # target sys/comp (autofilled)
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # VX,VY,VZ active only
        0, 0, 0,             # x,y,z pos (ignored)
        float(vx), float(vy), float(vz),
        0, 0, 0,             # ax,ay,az
        0, 0                 # yaw, yaw_rate
    )
    def send_once(): vehicle.send_mavlink(msg); vehicle.flush()
    if duration_s <= 0:
        send_once(); return
    t0 = time.time()
    while time.time() - t0 < duration_s:
        send_once()
        time.sleep(0.1)


def main():
    ap = argparse.ArgumentParser(description="Autonomy telemetry + control (Pi)")
    ap.add_argument("--conn", type=str, required=True,
                    help="Connection string (e.g., udp:0.0.0.0:14550 or /dev/ttyAMA0)")
    ap.add_argument("--baud", type=int, default=115200,
                    help="Baud for serial (ignored for UDP/TCP)")
    ap.add_argument("--hz", type=int, default=10, help="Telemetry poll rate (Hz)")
    ap.add_argument("--csv", type=str, default="", help="Optional CSV log path")
    ap.add_argument("--nofetch", action="store_true",
                    help="Skip requesting message intervals (use FC defaults)")
    args = ap.parse_args()

    baud = args.baud if args.conn.startswith("/dev/") else None
    print(f"[INFO] Connecting to {args.conn} ...")
    vehicle = connect(args.conn, baud=baud, wait_ready=True, timeout=60)
    print("[INFO] Connected.")

    if not args.nofetch:
        try:
            request_message_rates(vehicle, hz=max(5, args.hz))
            print("[INFO] Requested message intervals.")
        except Exception as e:
            print(f"[WARN] Could not request message intervals: {e}")

    tel = Telemetry()
    csv_writer = None
    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.DictWriter(csv_file, fieldnames=list(asdict(tel).keys()))
        csv_writer.writeheader()

    try:
        period = 1.0 / max(1, args.hz)
        while True:
            tel.stamp = time.time()
            tel.mode = vehicle.mode.name
            tel.armed = bool(vehicle.armed)
            tel.system_status = str(vehicle.system_status.state)
            tel.is_armable = bool(vehicle.is_armable)
            tel.last_hb = getattr(vehicle, "last_heartbeat", None)

            # GPS / Position
            gps = vehicle.gps_0
            tel.fix_type = getattr(gps, "fix_type", 0) or 0
            tel.sats = getattr(gps, "satellites_visible", 0) or 0

            loc_glob = vehicle.location.global_frame
            loc_rel = vehicle.location.global_relative_frame
            tel.lat = getattr(loc_glob, "lat", None)
            tel.lon = getattr(loc_glob, "lon", None)
            tel.alt_msl = getattr(loc_glob, "alt", None)
            tel.alt_rel = getattr(loc_rel, "alt", None)

            # Motion / Attitude
            att = vehicle.attitude
            tel.roll = getattr(att, "roll", None)
            tel.pitch = getattr(att, "pitch", None)
            tel.yaw = getattr(att, "yaw", None)
            vxyz = vehicle.velocity or (None, None, None)
            tel.vx, tel.vy, tel.vz = vxyz
            tel.groundspeed = getattr(vehicle, "groundspeed", None)
            tel.airspeed = getattr(vehicle, "airspeed", None)
            tel.heading = getattr(vehicle, "heading", None)

            # Sensors / Health
            rf = getattr(vehicle, "rangefinder", None)
            tel.rngf_dist = getattr(rf, "distance", None) if rf else None
            batt = vehicle.battery
            tel.batt_voltage = getattr(batt, "voltage", None)
            tel.batt_current = getattr(batt, "current", None)
            tel.batt_level = getattr(batt, "level", None)

            # EKF coarse OK: require 3D fix & many sats and armable
            tel.ekf_ok = (tel.fix_type >= 3 and tel.sats >= 6 and vehicle.is_armable)

            # Print compact status
            lat_s  = fmt(tel.lat, ".6f")
            lon_s  = fmt(tel.lon, ".6f")
            rel_s  = fmt(tel.alt_rel, ".1f")
            msl_s  = fmt(tel.alt_msl, ".1f")
            gs_s   = fmt(tel.groundspeed, ".1f")
            hdg_s  = fmt(tel.heading, ".0f")
            batt_s = fmt(tel.batt_voltage, ".2f")

            print(
                f"MODE {tel.mode:<10} ARM {tel.armed} | "
                f"GPS:{tel.fix_type}/{tel.sats} "
                f"LatLon:{lat_s},{lon_s} "
                f"Alt(rel/msl):{rel_s}/{msl_s} m | "
                f"GS:{gs_s} m/s Yaw:{hdg_s}° | "
                f"Batt:{batt_s}V"
            )


            if csv_writer:
                csv_writer.writerow(asdict(tel))
                csv_file.flush()

            time.sleep(period)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping…")

    finally:
        if csv_file:
            csv_file.close()
        vehicle.close()
        print("[INFO] Closed connection.")


if __name__ == "__main__":
    main()
