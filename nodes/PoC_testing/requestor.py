#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gray_transceiver.msg import GxTopicMetaInformation
from gray_transceiver.srv import GxRequest, GxOffer

class requester(object):

    def __init__(self):
        '''
        Constructor for requester class.
        '''
        rospy.init_node("requester")
        
    def run(self):
        '''
        Do the initial requests and then do nothing
        '''
        
        rospy.wait_for_service('gray_transceiver/requests')
        try:
            request = rospy.ServiceProxy('gray_transceiver/requests', GxRequest)
            odomRequest = GxTopicMetaInformation()
            odomRequest.description = "ODOM"
            odomRequest.type = "nav_msgs/Odometry"
            resp1 = request(odomRequest)
        except rospy.ServiceException, e:
            print "Odom service call failed: %s"%e

        try:
            request = rospy.ServiceProxy('gray_transceiver/requests', GxRequest)
            lidarRequest = GxTopicMetaInformation()
            lidarRequest.description = "LIDAR"
            lidarRequest.type = "sensor_msgs/LaserScan"
            resp2 = request(lidarRequest)
        except rospy.ServiceException, e:
            print "Lidar service call failed: %s"%e
        
        rospy.spin()


if __name__ == "__main__":
    Requester = requester()
    Requester.run()