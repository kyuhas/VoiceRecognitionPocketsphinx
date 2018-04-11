#!/usr/bin/env python

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math
import time

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Header, String

class voice_cmd_vel:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)

	# goals to use throughout program
	#TODO: replace with actual pose coordinates
	goal1 = PoseStamped(pose=Pose(1,1,1))
	goal2 = PoseStamped(pose=Pose(1,1,1))
	goal3 = PoseStamped(pose=Pose(1,1,1))
	goal4 = PoseStamped(pose=Pose(1,1,1))
	self.goals= [goal1, goal2, goal3, goal4]

	# create publishers
	self.moveBasePub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
	self.cmdVelPub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
		
        # subscribe to speech output and amcl pose
        rospy.Subscriber('recognizer/output', String, self.speechCb)
        self.poseSub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.set_pose)

	# robot pose
	self.robotPose = None

        time.sleep(1) #give publishers time to get set up
        
            
    def self.set_pose(self, data):
		self.robotPose = data.pose.pose
		
	def waitUntilGoalReached(self, goalPose):
		tolerance = 0.5
		while 1:
			if self.robotPose is not None:
				distance = self.getDistance(self.robotPose, goalPose)
				if distance <= tolerance:
					break

	def getDistance(self, robot, goal):
		return math.sqrt(math.pow(robot.position.x - goal.position.x, 2) + math.pow(robot.position.y - goal.position.y, 2))
		
	def goToGoalPosition(self, goalNum):
		goal = self.goals[goalNum - 1] #start at 0 in array
		goal.header = Header(frame_id = 'map', stamp = rospy.get_rostime())
		self.moveBasePub.publish(goal)
		self.waitUntilGoalReached(currentCorner.pose) #TODO: if you hear "Stop", stop!	
	
	def sendCmdVel(self, linear, angular):
		twistMsg = Twist()
		twistMsg.linear.x = linear
		twistMsg.angular.z = angular
		self.cmdVelPub.publish(twistMsg)
    
    def speechCb(self, msg):
        if msg.data.find("stop") > -1:
            # stop the robot
			self.sendCmdVel(0,0)
        elif msg.data.find("turn left") > -1:
            # turn robot left
			self.sendCmdVel(0, -0.2)
        elif msg.data.find("turn right") > -1:    
            # turn robot right
			self.sendCmdVel(0, 0.2)
        elif msg.data.find("move forward") > -1:
            # move robot forward
			self.sendCmdVel(0.5, 0)
        elif msg.data.find("move backward") > -1:    
            # move robot backward
			self.sendCmdVel(-0.5, 0)
        elif msg.data.find("go to position") > -1:
            # move to whatever position is designated in the hyp (i.e. one, two, three, etc)
			if msg.data.find("one") > -1:
				self.goToGoalPosition(1)
			elif msg.data.find("two") > -1:
				self.goToGoalPosition(2)
			elif msg.data.find("three") > -1:
				self.goToGoalPosition(3)
			elif msg.data.find("four") > -1:
				self.goToGoalPosition(4)

    def cleanup(self):
        self.sendCmdVel(0,0)

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    try:
        voice_cmd_vel()
    except:
        pass

