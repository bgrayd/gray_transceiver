#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

'''
This robot drives forward and uses its laser range finder to avoid obstacles.
'''

class sandbox_2(object):

	def __init__(self):
		'''
		Constructor for sandbox_2 class.
		'''
		rospy.init_node("sandbox_2")									# initialize this as a ROS node named 'sandbox_2'
		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist)			# create publisher that can be used to publish twist messages over /cmd_vel topic
		rospy.Subscriber("base_scan", LaserScan, self.laser_callback) 	# subscribe to /base_scan topic using self.laser_callback as the callback function

		self.obstacle_detected = False 	# this variable will keep track of whether or not the robot currently sees an obstacle
		self.rightWall = False #keep track of if it can see something on the right side

	def laser_callback_basic(self, data):
		'''
		This function gets called everytime a message is published over the /base_scan topic.
		'''
		
		comfort_threshold = 0.5 # how close to an obstacle we are comfortable getting

		flag = False
		# for each laser value in data.ranges
		#for i in xrange(0, len(data.ranges)):

		#just look at the 1/4 of the view in front of the robot
		middlePoint = len(data.ranges)/2
		eigthPoints = len(data.ranges)/8
		for i in xrange(middlePoint - eigthPoints, middlePoint + eigthPoints): 
			# if the range value i is less than comfort_threshold, there is an obstacle in front of robot
			if data.ranges[i] < comfort_threshold:
				flag = True
		
		self.obstacle_detected = flag		
		
	#right wall follower
	def laser_callback(self, data):
		'''
		This function gets called everytime a message is published over the /base_scan topic.
		'''
		
		comfort_threshold = 0.5 # how close to an obstacle we are comfortable getting

		flag = False
		# for each laser value in data.ranges
		#for i in xrange(0, len(data.ranges)):

		#just look at the 1/4 of the view in front of the robot
		middlePoint = len(data.ranges)/2
		eigthPoints = len(data.ranges)/8
		for i in xrange(middlePoint - eigthPoints, middlePoint + eigthPoints): 
			# if the range value i is less than comfort_threshold, there is an obstacle in front of robot
			if data.ranges[i] < comfort_threshold:
				flag = True

		rightMidPoint = len(data.ranges)*1/4
		flag2 = False
		for i in xrange(rightMidPoint - eigthPoints, rightMidPoint + eigthPoints): 
			# if the range value i is less than comfort_threshold, there is an obstacle in front of robot
			if data.ranges[i] < comfort_threshold*2:
				flag2 = True
		
		self.obstacle_detected = flag
		self.rightWall = flag2

		
		
	def run(self):
		'''
		When this function is called, commands will continuously be sent to the
		 simulated robot that will make the robot drive mostly forward.
		The obstacle detected variables is used to determine when to avoid obstacles in the robot's path.
		'''
		while not rospy.is_shutdown():
			twist = Twist()		# create an empty twist message

			if self.obstacle_detected:
				# if an obstacle is detected, just turn
				twist.angular.z = 0.25
				self.cmd_vel_pub.publish(twist)
			else:
				# otherwise, go forward
				twist.linear.x = 0.75   # forward
				twist.angular.z = 0#0.05  # make it turn a little bit
				if not self.rightWall:
					twist.linear.x = 0.3 #move slower
					twist.angular.z = -0.3

			self.cmd_vel_pub.publish(twist)		# publish our twist message to cmd_vel topic

			rospy.sleep(0.1)					# sleep briefly so ROS doesn't die


if __name__ == "__main__":
	sandbox = sandbox_2()
	sandbox.run()
