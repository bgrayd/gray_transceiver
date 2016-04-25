#!/usr/bin/env python

import rospy
from Queue import *
from gray_transceiver.msg import GxMetaTopic, twistCommand
from geometry_msgs.msg import Twist

hertz = 5

class cmdVelRepub(object):
    def __init__(self):
        rospy.init_node("cmd_vel_repuber")

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("gray_transceiver/metatopic", GxMetaTopic, self.meta_sub)

        self.thisRobotName = ''

        self.twistQueue = Queue(1000)

    def meta_sub(self, data):
        self.thisRobotName = data.myName
        if(data.type == "gray_transceiver/twistCommand"):
            if(data.description == "forward"):
                rospy.Subscriber(data.name, twistCommand, self.twistCmd_sub)

    def twistCmd_sub(self, data):
        if(data.targetName == self.thisRobotName):
            cmd_vel = Twist()
            cmd_vel.linear = data.linear
            cmd_vel.angular = data.angular
            for each in range(0, data.seconds*hertz):
                self.twistQueue.put(cmd_vel)

    def run(self):
        rate = rospy.Rate(hertz)
        while not rospy.is_shutdown():
            if not self.twistQueue.empty():
                message = self.twistQueue.get()
                self.cmd_vel_pub.publish(message)
            rate.sleep()