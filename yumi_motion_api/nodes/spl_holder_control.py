#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

# Global variable for serial connection
ser = None

def arduino_serial():
    global ser

    # Initialize ROS node
    rospy.init_node('arduino_serial_node', anonymous=True)

    # Define serial port settings
    serial_port = rospy.get_param("~serial_port", "/dev/ttyUSB0")  # Change this according to your setup
    baud_rate = rospy.get_param("~baud_rate", 9600)

    # Initialize serial connection
    ser = serial.Serial(serial_port, baud_rate, timeout=1)

    # Define ROS publisher and subscriber
    pub = rospy.Publisher('arduino_data', String, queue_size=10)

    # Define ROS service
    rospy.Service('latch_spl_holder', Trigger, latch_spl_holder)
    rospy.Service('release_spl_holder', Trigger, release_spl_holder)

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline().strip()
            rospy.loginfo("Received from Arduino: %s", line)
            pub.publish(line)
        rate.sleep()

    # Close serial connection before exiting
    ser.close()

def latch_spl_holder(req):
    global ser

    # This function latches the SPL holder
    command = "latch\n"  # Command to send to Arduino
    rospy.loginfo("Latching SPL holder...")
    ser.write(command.encode('utf-8'))
    return TriggerResponse(success=True, message="SPL holder latched")

def release_spl_holder(req):
    global ser

    # This function releases the SPL holder
    command = "release\n"  # Command to send to Arduino
    rospy.loginfo("Releasing SPL holder...")
    ser.write(command.encode('utf-8'))
    return TriggerResponse(success=True, message="SPL holder released")

if __name__ == '__main__':
    try:
        arduino_serial()
    except rospy.ROSInterruptException:
        pass
