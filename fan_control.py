# fan_control.py (verbose)
from gpiozero import DigitalOutputDevice
import time
from gpiozero import Device
from gpiozero.pins.rpigpio import RPiGPIOFactory
Device.pin_factory = RPiGPIOFactory()

FAN_PIN = 14
ACTIVE_LOW = False           # set True if your transistor/hat pulls fan ON when GPIO is LOW
from gpiozero import DigitalOutputDevice
fan = DigitalOutputDevice(FAN_PIN, active_high=not ACTIVE_LOW, initial_value=False)

ON_TEMP = 45.0               # °C
OFF_TEMP = 40.0              # °C
POLL_S = 2.0


def get_temp_c():
    with open("/sys/class/thermal/thermal_zone0/temp") as f:
        return int(f.read().strip()) / 1000.0

last_state = None
try:
    print(f"[init] pin={FAN_PIN} on={ON_TEMP}°C off={OFF_TEMP}°C poll={POLL_S}s active_low={ACTIVE_LOW}")
    while True:
        t = get_temp_c()
        # hysteresis
        if t >= ON_TEMP and not fan.value:
            fan.on()
        elif t <= OFF_TEMP and fan.value:
            fan.off()

        if fan.value != last_state:
            print(f"[state] temp={t:.1f}°C fan={'ON' if fan.value else 'OFF'}")
            last_state = fan.value

        # periodic heartbeat
        if int(time.time()) % 10 == 0:
            print(f"[tick]  temp={t:.1f}°C fan={'ON' if fan.value else 'OFF'}")
            time.sleep(1)  # avoid spamming every second at the same timestamp
        else:
            time.sleep(POLL_S)
except KeyboardInterrupt:
    fan.off()
    print("\n[exit] fan OFF")
