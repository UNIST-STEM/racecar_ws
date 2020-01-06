#!/usr/bin/python

# node for turning gamepad inputs into drive commands
import math
import rospy
from sensor_msgs.msg import Joy
from racecar_ws.msg import drive_msg

print("Complete to open Gamepad Node")

# gamepad callback
def joy_callback(msg):
    global drive_pub, drive, X_SCALE, Y_SCALE
    drive = drive_msg()
    if msg.buttons[1]==1:
	drive.velocity = 0
    else: 
	drive.drive_angle = 255*msg.axes[3] #left && right
	drive.velocity = 255*msg.axes[1] #front && back




    drive_pub.publish(drive)

# init ROS
rospy.init_node('gamepad')
drive_pub = rospy.Publisher('/gamepad_drive', drive_msg, queue_size=1)
rospy.Subscriber('/joy', Joy, joy_callback)

# wait before shutdown
rospy.spin()
