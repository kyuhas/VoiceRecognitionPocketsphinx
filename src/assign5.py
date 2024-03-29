#!/usr/bin/env python

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math
import time

from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Header, String

class voice_cmd_vel():
	def __init__(self):
		rospy.on_shutdown(self.cleanup)
		# create publisher
		self.cmdVelPub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
		
		# subscribe to speech output and amcl pose
		rospy.Subscriber('recognizer/output', String, self.speechCb)
		self.cmd = "stop"

		time.sleep(1) #give publisher time to get set up
		rospy.spin()

	def executeCmd(self, cmdStr):
		if "stop" in cmdStr:
			# stop the robot
			self.cmd = "stop"
			self.sendCmdVel(0,0)
		elif "turn left" in cmdStr:
			# turn robot left
			self.cmd = "turn left"
			self.sendCmdVel(0, -0.2)
		elif "turn right" in cmdStr:    
			# turn robot right
			self.cmd = "turn right"
			self.sendCmdVel(0, 0.2)
		elif "move forward" in cmdStr:
			# move robot forward
			self.cmd = "move forward"
			self.sendCmdVel(0.5, 0)
		elif "move backward" in cmdStr:    
			# move robot backward
			self.cmd = "move backward"
			self.sendCmdVel(-0.5, 0)
	
	def sendCmdVel(self, linear, angular):
		twistMsg = Twist()
		twistMsg.linear.x = linear
		twistMsg.angular.z = angular
		self.cmdVelPub.publish(twistMsg)

	def speechCb(self, msg):
		self.executeCmd(msg.data)

    	def cleanup(self):
        	self.sendCmdVel(0,0)

	#TODO: uncomment this out once 
	#def run(self):
		#count = 1
		# keep calling the same command while the robot is running
		# however, limit it with a counter so that it doesn't do it too much
		#while True:
			#if count%100 == 0:
				#self.executeCmd(self.cmd)
				#count = 1
			#else:
				#count = count + 1

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    try:
        voice_cmd_vel()#.run()
    except:
        pass

