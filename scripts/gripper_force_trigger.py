#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from hx711 import HX711
import time

# Parameters
DATA_PIN = rospy.get_param("~data_pin", 31)         # Physical pin (e.g. 31 -> GPIO6)
CLOCK_PIN = rospy.get_param("~clock_pin", 29)       # Physical pin (e.g. 29 -> GPIO5)
SCALE_FACTOR = rospy.get_param("~scale_factor", 20938.0)  # Calibration factor
FORCE_CUTOFF = rospy.get_param("~force_cutoff", 80.0)      # Maximum allowable force in Newtons
FILTER_SIZE = rospy.get_param("~filter_size", 3)     # Size of moving average filter
FREQ = rospy.get_param("~publish_frequency", 50.0)   # Publishing frequency (Hz)

rospy.init_node("gripper_force_trigger", anonymous=True)
pub = rospy.Publisher("gripper_force_trigger", Float32, queue_size=10)
rate = rospy.Rate(FREQ)

# Initialize HX711
hx = HX711(dout=DATA_PIN, pd_sck=CLOCK_PIN, line_map_name='RPI_5')
hx.set_reference_unit(SCALE_FACTOR)
hx.tare()

# Filter buffer
force_readings = [0.0] * FILTER_SIZE
filter_index = 0

rospy.loginfo("Force trigger node ready.")

while not rospy.is_shutdown():
    # Read and convert
    force = hx.get_weight()

    # Apply cutoff and bounds
    force = min(max(force, 0.0), FORCE_CUTOFF)

    # Store in circular buffer
    force_readings[filter_index] = force
    filter_index = (filter_index + 1) % FILTER_SIZE

    # Moving average
    filtered_force = sum(force_readings) / FILTER_SIZE

    # Publish
    msg = Float32()
    msg.data = filtered_force
    pub.publish(msg)

    rate.sleep()
