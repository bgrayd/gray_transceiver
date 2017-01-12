#!/usr/bin/env python

import rospy
import subprocess
import threading
import tf
from gray_transceiver.msg import GxMetaTopic
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry



#subprocess.Popen(["rosrun","gmapping", "slam_gmapping", "scan:=base_scan"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)

class gmapping_launcher(object):

    def __init__(self):
        '''
        Constructor for requestor class.
        '''
        rospy.init_node("hectormapping_launcher")
        rospy.Subscriber("gray_transceiver/metatopic", GxMetaTopic, self.metaTopic_callback)
        self.subprocs = []
        self.pub = rospy.Publisher("/robot_0/base_scan", LaserScan, queue_size=10)
        rospy.Subscriber("/base_scan", LaserScan, self.lidar_callback)#"/"+namespace+
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def lidar_callback(self, data):
        self.pub.publish(data)

    def odom_callback(self, data):
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "odom", "origin" )


    def metaTopic_callback(self, data):
        if data.type == "sensor_msgs/LaserScan":
            namePieces = data.name.split("/")#"scan:="+data.name, "map_metadata:="+namePieces[0]+"/map_metadata", "map:="+namePieces[0]+"/map",
            rospy.set_param(namePieces[0]+"/robotName", namePieces[0])
            # self.subprocs.append(subprocess.Popen(["rosrun","gmapping", "slam_gmapping", "__ns:="+namePieces[0], "scan:=reframed_scan","_map_frame:="+namePieces[0]+"_map", "_base_frame:="+namePieces[0]+"_base_link", "_odom_frame:="+namePieces[0]+"_odom" ], stdin=subprocess.PIPE, stdout=subprocess.PIPE))
            self.subprocs.append(subprocess.Popen(["rosrun","gray_transceiver", "lidar_remapper2.py", "__ns:="+namePieces[0]], stdin=subprocess.PIPE, stdout=subprocess.PIPE))#,"robotName:="+namePieces[0]

            

        
    def run(self):
        # rospy.spin()
        listener = tf.TransformListener()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                frame1 = 'odom'
                frame2 = 'base_footprint'
                (trans,rot) = listener.lookupTransform(frame1, frame2, rospy.Time.now())
                br=tf.TransformBroadcaster()
                br.sendTransform(trans,rot, rospy.Time.now(), "robot_0_"+frame2, "robot_0_"+frame1)
                br=tf.TransformBroadcaster()
                br.sendTransform(trans,rot, rospy.Time.now(), "robot_1_"+frame2, "robot_1_"+frame1)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            try:
                frame1 = 'base_footprint'
                frame2 = 'base_link'
                (trans,rot) = listener.lookupTransform(frame1, frame2, rospy.Time.now())
                br=tf.TransformBroadcaster()
                br.sendTransform(trans,rot, rospy.Time.now(), "robot_0_"+frame2, "robot_0_"+frame1)
                br=tf.TransformBroadcaster()
                br.sendTransform(trans,rot, rospy.Time.now(), "robot_1_"+frame2, "robot_1_"+frame1)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            try:
                frame1 = 'base_link'
                frame2 = 'base_laser_link'
                (trans,rot) = listener.lookupTransform(frame1, frame2, rospy.Time.now())
                br=tf.TransformBroadcaster()
                br.sendTransform(trans,rot, rospy.Time.now(), "robot_0_"+frame2, "robot_0_"+frame1)
                br=tf.TransformBroadcaster()
                br.sendTransform(trans,rot, rospy.Time.now(), "robot_1_"+frame2, "robot_1_"+frame1)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
        for each in self.subprocs:
            each.terminate()

if __name__ == '__main__':
    try:
        newNode = gmapping_launcher()
        newNode.run()
    except rospy.ROSInterruptException:
        pass
