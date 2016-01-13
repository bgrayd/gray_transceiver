#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class requestor(object):

    def __init__(self):
        '''
        Constructor for requestor class.
        '''
        rospy.init_node("requestor")
        self.request_pub = rospy.Publisher("gray_transceiver/requests", String, queue_size=10)

        
    def run(self):
        '''
        Do the initial requests and then do nothing
        '''
        rospy.sleep(15)
        # message = String()
        # message.data = "ODOM"
        # self.request_pub.publish(message)
        # print("published ODOM")
        # message.data = "LIDAR"
        # self.request_pub.publish(message)
        # print("Published LIDAR")

        while not rospy.is_shutdown():
            message = String()
            message.data = "ODOM~nav_msgs/Odometry"
            self.request_pub.publish(message)
            print("published ODOM")
            message.data = "LIDAR~sensor_msgs/LaserScan"
            self.request_pub.publish(message)
            print("Published LIDAR")
            rospy.spin()#sleep(0.5)


if __name__ == "__main__":
    Requestor = requestor()
    Requestor.run()