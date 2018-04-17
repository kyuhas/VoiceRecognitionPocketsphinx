#!/usr/bin/env python

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math
import time

from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Header, String

from sound_play.libsoundplay import SoundClient

class voice_cmd_vel():
	def __init__(self):
		rospy.on_shutdown(self.cleanup)
		# create publisher
		self.cmdVelPub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)		

		# subscribe to speech output
		self.sub = rospy.Subscriber('recognizer/output', String, self.speechCb)
		self.cmd = "stop"
		self.verifyCmd = "stop"
		# create soundhandle
		self.soundhandle = SoundClient()

		time.sleep(1) #give publisher time to get set up

	def executeCmd(self, cmdStr):
		# as of now, the only command it asks verification of is "move backward"
		if "stop" in cmdStr:
			# stop the robot
			self.cmd = "stop"
			self.soundhandle.say('stop')
			self.sendCmdVel(0,0)
		elif "turn left" in cmdStr:
			# turn robot left
			self.cmd = "turn left"
			self.soundhandle.say('turning left')
			self.sendCmdVel(0, 0.5)
		elif "turn right" in cmdStr:    
			# turn robot right
			self.cmd = "turn right"
			self.soundhandle.say('turning right')
			self.sendCmdVel(0, -0.5)
		elif "move forward" in cmdStr:
			# move robot forward
			self.cmd = "move forward"
			self.soundhandle.say('moving forward')
			self.sendCmdVel(0.3, 0)
		elif "move backward" in cmdStr:    
			# move robot backward
			# first, stop the robot and have it ask to confirm
			self.verifyCmd = "move backward"
			self.soundhandle.say('Are you sure you want me to move backward?')
		elif "yes" in cmdStr:
			self.soundhandle.say('Okay, I will now ' + str(self.verifyCmd))
			self.cmd = "move backward"
			self.sendCmdVel(-0.3, 0)
	
	def sendCmdVel(self, linear, angular):
		twistMsg = Twist()
		twistMsg.linear.x = linear
		twistMsg.angular.z = angular
		self.cmdVelPub.publish(twistMsg)

	def speechCb(self, msg):
		self.executeCmd(msg.data)

    	def cleanup(self):
        	self.sendCmdVel(0,0)
 
	def run(self):
		# keep calling the same command while the robot is running
		r = rospy.Rate(10)
		while True:
			self.executeCmd(self.cmd)
			r.sleep()

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    try:
        voice_cmd_vel().run()
    except:
        pass

