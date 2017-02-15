#!/usr/bin/env python

import rospy, roslaunch
from Queue import *
from gray_transceiver.msg import GxMetaTopic, twistCommand
from geometry_msgs.msg import Twist

hertz = 5

class cmdVelRepub(object):
    def __init__(self):
        rospy.init_node("cmd_vel_repuber")

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        rospy.Subscriber("gray_transceiver/metatopic", GxMetaTopic, self.meta_sub)

        self.thisRobotName = ''

        self.seenIds = []

        self.twistQueue = Queue(1000)

        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        self.launchGx()
        rospy.on_shutdown(self.killGx)

    def launchGx(self):
        package = "gray_transceiver"
        executable = "gray_transceiver_main.py"
        nodeNamespace = "/"
        nodeName = "gray_transceiver_main"

        arguments = ""

        gxNode = roslaunch.core.Node(package, executable,name=nodeName, namespace=nodeNamespace, args=arguments)
        self.gxNode = self.launch.launch(gxNode)

    def killGx(self):
        self.gxNode.stop()

    def meta_sub(self, data):
        self.thisRobotName = data.myName
        if(data.type == "gray_transceiver/twistCommand"):
            rospy.Subscriber(data.name, twistCommand, self.twistCmd_sub)
            # if(data.description == "forward"):
                # rospy.Subscriber(data.name, twistCommand, self.twistCmd_sub)

    def twistCmd_sub(self, data):
        print(self.thisRobotName)
        print(data.name)
        if ((str(data.name) == str(self.thisRobotName)) or (str(data.name) == "all")) and (str(data.id) not in self.seenIds):
            self.seenIds.append(str(data.id))
            cmd_vel = Twist()
            cmd_vel.linear = data.linear
            cmd_vel.angular = data.angular
            # self.cmd_vel_pub.publish(cmd_vel)
            for each in range(0, int(data.seconds*hertz)):
                self.twistQueue.put(cmd_vel)
            if data.angular.x == 1:
                self.killGx()

    def run(self):
        rate = rospy.Rate(hertz)
        while not rospy.is_shutdown():
            if not self.twistQueue.empty():
                message = self.twistQueue.get()
                self.cmd_vel_pub.publish(message)
            rate.sleep()


if __name__ == "__main__":
    node = cmdVelRepub()
    node.run()
