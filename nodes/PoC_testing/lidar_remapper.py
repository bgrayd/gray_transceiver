#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

namespace = rospy.get_param("robotName", "")

class frame_remapper(object):
    def __init__(self):
        self.firstOdom = True
        self.angle=0
        rospy.init_node("frame_remaper")
        self.pub = rospy.Publisher("reframed_scan", LaserScan, queue_size=10)
        rospy.Subscriber("LIDAR", LaserScan, self.lidar_callback)#"/"+namespace+
        rospy.Subscriber("ODOM", Odometry, self.odom_callback)
        rospy.Subscriber("angleZ", Float64, self.angle_callback)

    def lidar_callback(self, data):
        data.header.frame_id = "base_laser_link"#namespace+"_base_link"
        data.header.stamp = rospy.Time.now()
        self.pub.publish(data)

    def odom_callback(self, data):
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, 0, self.angle), rospy.Time.now(), namespace+"_map", "world" )
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "map", "world" )
        br = tf.TransformBroadcaster()#(0, 0, msg.theta)
        br.sendTransform((data.pose.pose.position.x, data.pose.pose.position.y, 0),(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w), rospy.Time.now(), namespace+"_base_link", namespace+"_odom" )

    def angle_callback(self, data):
        self.angle = data

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        newNode = frame_remapper()
        newNode.run()
    except rospy.ROSInterruptException:
        pass