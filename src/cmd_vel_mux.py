#!/usr/bin/python
# license removed for brevity

import rospy
import time

from racecar_ws.msg import drive_msg
from sensor_msgs.msg import Joy

mux_mode = ''

print("Complete to open cmd_vel_mux Node") 

def joy_callback(msg):
	global mux_mode, mux_out_pub

	if msg.buttons[4] == 1:
		mux_mode = 'gamepad' #LB >> joy

	elif msg.buttons[5] ==1:
		mux_mode = 'autonomy' #RB >> auto
		#print("mux_mode: autonomy")

	else:
		mux_mode = ''
		#print("mux_mode: standby")
		drive=drive_msg()
		drive.velocity=0
		drive.drive_angle=0
		mux_out_pub.publish(drive)

def gamepad_drive_callback(msg):
	global mux_mode, mux_out_pub
	if mux_mode == 'gamepad':
		mux_out_pub.publish(msg)

def drive_callback(msg):
	global mux_mode, mux_out_pub
	if mux_mode == 'autonomy':
		mux_out_pub.publish(msg)

#init ROS
rospy.init_node('cmd_vel_mux')
mux_out_pub = rospy.Publisher('/mux_out', drive_msg, queue_size=1)
rospy.Subscriber('/gamepad_drive', drive_msg, gamepad_drive_callback)
rospy.Subscriber('/drive', drive_msg, drive_callback)
rospy.Subscriber('/joy', Joy, joy_callback)

rospy.spin()
	


