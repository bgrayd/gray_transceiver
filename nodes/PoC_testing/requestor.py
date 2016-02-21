#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gray_transceiver.msg import GxRequest

class requester(object):

    def __init__(self):
        '''
        Constructor for requester class.
        '''
        rospy.init_node("requester")
        self.request_pub = rospy.Publisher("gray_transceiver/requests", GxRequest, queue_size=10)

        
    def run(self):
        '''
        Do the initial requests and then do nothing
        '''
        rospy.sleep(15)

        while not rospy.is_shutdown():
            message = GxRequest()
            message.description = "ODOM"
            message.type = "nav_msgs/Odometry"
            self.request_pub.publish(message)
            print("published ODOM")
            message.description = "LIDAR"
            message.type = "sensor_msgs/LaserScan"
            self.request_pub.publish(message)
            print("Published LIDAR")
            rospy.spin()#sleep(0.5)


if __name__ == "__main__":
    Requester = requester()
    Requester.run()