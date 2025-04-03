#!/usr/bin/env python3
import time
from hx711 import HX711

# GPIO pins (adjust if needed)
DATA_PIN = 5
CLOCK_PIN = 6

# Initialize HX711
hx = HX711(dout_pin=DATA_PIN, pd_sck_pin=CLOCK_PIN)
hx.set_reading_format("MSB", "MSB")
hx.reset()

print("Remove all weight from scale. Taring in 3 seconds...")
time.sleep(3)
hx.tare()
print("Tare complete.")

print("Place a known weight and observe raw readings.")
print("Press CTRL+C to stop.")

try:
    while True:
        raw_val = hx.get_weight(5)  # Average over 5 readings
        print(f"Raw HX711 Reading: {raw_val:.2f}")
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\nDone.")
    try:
        hx.power_down()
    except:
        pass
