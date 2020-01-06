#!/usr/bin/python 

# node for turning gamepad inputs into drive commands 

import rospy 
import time
from racecar_ws.msg import drive_msg



def cmd_vel_mux():
	pub = rospy.Publisher("/mux_out", drive_msg, queue_size=1)
	rospy.init_node("cmd_vel_mux", anonymous=True)
	drive=drive_msg()
	#rate = rospy.Rate(10)
	
	drive.acceleration=0
	
	drive.drive_angle = 255
	drive.velocity = 255
	pub.publish(drive)
   	time.sleep(5.0)
	
	drive.drive_angle = 0
	drive.velocity = 0
	pub.publish(drive)
   	time.sleep(1.0)	

	drive.drive_angle = -255
	drive.velocity = -255
	pub.publish(drive)
   	time.sleep(5.0)
	
		
	drive.drive_angle = 0
	drive.velocity = 0
	pub.publish(drive)
   	time.sleep(1.0)

if __name__ == '__main__':
	try:
		cmd_vel_mux()
	except rospy.ROSInterruptException:
        	pass

