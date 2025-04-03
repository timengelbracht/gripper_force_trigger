#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from hx711 import HX711
import signal
import sys

# Initialize ROS node
rospy.init_node('pi_force_reader')

# Load ROS parameters
DATA_PIN = rospy.get_param('~data_pin', 5)
CLOCK_PIN = rospy.get_param('~clock_pin', 6)
REFERENCE_UNIT = rospy.get_param('~reference_unit', 20938)
FORCE_CUTOFF = rospy.get_param('~force_cutoff', 80.0)
FILTER_SIZE = rospy.get_param('~filter_size', 3)
PUBLISH_HZ = rospy.get_param('~publish_rate', 20)

# ROS publisher
pub = rospy.Publisher('/gripper_force_trigger', Float32, queue_size=10)
rate = rospy.Rate(PUBLISH_HZ)

# HX711 Init
hx = HX711(dout_pin=DATA_PIN, pd_sck_pin=CLOCK_PIN)
hx.set_reading_format("MSB", "MSB")
hx.set_reference_unit(REFERENCE_UNIT)
hx.reset()
hx.tare()
rospy.loginfo("HX711 ready and tared.")

# Graceful exit
def clean_and_exit(sig, frame):
    rospy.loginfo("Shutting down HX711 node...")
    hx.power_down()
    sys.exit(0)

signal.signal(signal.SIGINT, clean_and_exit)

# Initialize filter buffer
force_buffer = [0.0] * FILTER_SIZE
index = 0

# Main loop
while not rospy.is_shutdown():
    try:
        val = hx.get_weight(5)  # average over 5 samples
        force = val * 9.81 / 1000.0  # grams to Newtons

        # Clamp force to cutoff range
        force = max(0.0, min(force, FORCE_CUTOFF))

        # Update buffer and compute moving average
        force_buffer[index] = force
        index = (index + 1) % FILTER_SIZE
        filtered_force = sum(force_buffer) / FILTER_SIZE

        pub.publish(filtered_force)
        hx.power_down()
        hx.power_up()
        rate.sleep()
    except Exception as e:
        rospy.logwarn(f"HX711 read failed: {e}")
        rate.sleep()
