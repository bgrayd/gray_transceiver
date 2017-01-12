#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

namespace = rospy.get_param("robotName", "")

class frame_remapper(object):
    def __init__(self):
        self.firstOdom = True
        rospy.init_node("frame_remaper2")
        self.pub = rospy.Publisher("/robot_1/base_scan", LaserScan, queue_size=10)
        rospy.Subscriber("LIDAR", LaserScan, self.lidar_callback)#"/"+namespace+
        rospy.Subscriber("ODOM", Odometry, self.odom_callback)

    def lidar_callback(self, data):
        data.header.frame_id = "robot_1_base_laser_link"
        data.header.stamp = rospy.Time.now()
        self.pub.publish(data)

    def odom_callback(self, data):
        # br = tf.TransformBroadcaster()
        # br.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "robot_1_odom", "origin" )

        # br = tf.TransformBroadcaster()
        # br.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "odom", "origin" )

        br = tf.TransformBroadcaster()#(0, 0, msg.theta)
        # br.sendTransform((data.pose.pose.position.x, data.pose.pose.position.y, 0),(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w), rospy.Time.now(), "robot_1_basefootprint", namespace+"_odom" )
        br.sendTransform((0, 0, 0),(0, 0, 0, 1), rospy.Time.now(), "robot_1_basefootprint", "robot_1_odom" )

        # br=tf.TransformBroadcaster()
        # br.sendTransform((0, 0, 0),(0, 0, 0, 1), rospy.Time.now(), "robot_1_base_link", "robot_1_basefootprint")

        # br=tf.TransformBroadcaster()
        # br.sendTransform((0, 0, 0.45),(0, 0, 0, 1), rospy.Time.now(), "robot_1_base_laser_link", "robot_1_base_link")


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        newNode = frame_remapper()
        newNode.run()
    except rospy.ROSInterruptException:
        pass