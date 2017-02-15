#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gray_transceiver.msg import GxTopicMetaInformation, GxMetaTopic, twistCommand
from gray_transceiver.srv import GxRequest, GxOffer

twist_all = twistCommand()
twist_all.name="all"
twist_all.angular.z = 0.05
twist_all.seconds = 8

twist_wait = twistCommand()
twist_wait.name = "all"
twist_wait.seconds = 4

twist_first = twistCommand()
twist_first.angular.z = -0.05
twist_first.seconds = 14

twist_second = twistCommand()
twist_second.angular.z = 0.05
twist_second.seconds = 14

twist_third_1 = twistCommand()
twist_third_1.angular.z = 0.05
twist_third_1.seconds = 7

twist_third_2 = twistCommand()
twist_third_2.angular.x = 1
twist_third_2.angular.z = -0.05
twist_third_2.seconds = 7


class requester(object):

    def __init__(self):
        '''
        Constructor for requester class.
        '''
        rospy.init_node("requester")
        self.robotNames = []
        rospy.Subscriber("gray_transceiver/metatopic", GxMetaTopic, self.meta_sub)
        self.twistCmd_pub = rospy.Publisher("/move_cmd", twistCommand, queue_size=10)

    def meta_sub(self, data):
        nameParts = data.name.split("/")
        firstParts = nameParts[0].split("_")
        name = firstParts[1]
        self.robotNames.append(name)

    def run(self):
        '''
        Do the initial requests and then do nothing
        '''

        rospy.wait_for_service('gray_transceiver/offers')
        try:
            offer = rospy.ServiceProxy('gray_transceiver/offers', GxOffer)
            odomOffer = GxTopicMetaInformation()
            odomOffer.description = "move_cmd"
            odomOffer.type = "gray_transceiver/twistCommand"
            resp3 = offer(odomOffer, "/move_cmd")
        except rospy.ServiceException, e:
            print "Odom offer service call failed: %s"%e

        rospy.wait_for_service('gray_transceiver/requests')
        try:
            request = rospy.ServiceProxy('gray_transceiver/requests', GxRequest)
            odomRequest = GxTopicMetaInformation()
            odomRequest.description = "kinectCamera/compressed"
            odomRequest.type = "sensor_msgs/CompressedImage"
            resp1 = request(odomRequest)
        except rospy.ServiceException, e:
            print "Odom request service call failed: %s"%e

        try:
            request = rospy.ServiceProxy('gray_transceiver/requests', GxRequest)
            odomRequest = GxTopicMetaInformation()
            odomRequest.description = "kinectCamera/camera_info"
            odomRequest.type = "sensor_msgs/CameraInfo"
            #resp1 = request(odomRequest)
        except rospy.ServiceException, e:
            print "Odom request service call failed: %s"%e

        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and len(self.robotNames)<1:
            rate.sleep()


        twist_first.name = self.robotNames[0]
        twist_second = self.robotNames[0]
        twist_third_1 = self.robotNames[0]
        twist_third_2 = self.robotNames[0]

        self.twistCmd_pub.publish(twist_all)
        self.twistCmd_pub.publish(twist_wait)
        self.twistCmd_pub.publish(twist_first)
        self.twistCmd_pub.publish(twist_second)
        self.twistCmd_pub.publish(twist_third_1)
        self.twistCmd_pub.publish(twist_third_2)



if __name__ == "__main__":
    Requester = requester()
    Requester.run()