#!/usr/bin/env python

import rospy
import subprocess
import threading
import tf
from gray_transceiver.msg import GxMetaTopic



#subprocess.Popen(["rosrun","gmapping", "slam_gmapping", "scan:=base_scan"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)

class gmapping_launcher(object):

    def __init__(self):
        '''
        Constructor for requestor class.
        '''
        rospy.init_node("gmapping_launcher")
        rospy.Subscriber("gray_transceiver/metatopic", GxMetaTopic, self.metaTopic_callback)
        self.subprocs = []

    def metaTopic_callback(self, data):
        if data.type == "sensor_msgs/LaserScan":
            namePieces = data.name.split("/")#"scan:="+data.name, "map_metadata:="+namePieces[1]+"/map_metadata", "map:="+namePieces[1]+"/map",
            rospy.set_param(namePieces[1]+"/robotName", namePieces[1])
            self.subprocs.append(subprocess.Popen(["rosrun","gmapping", "slam_gmapping", "__ns:="+namePieces[1], "scan:=reframed_scan","_map_frame:="+namePieces[1]+"_map", "_base_frame:="+namePieces[1]+"_base_link", "_odom_frame:="+namePieces[1]+"_odom" ], stdin=subprocess.PIPE, stdout=subprocess.PIPE))
            self.subprocs.append(subprocess.Popen(["rosrun","gray_transceiver", "lidar_remapper.py", "__ns:="+namePieces[1]], stdin=subprocess.PIPE, stdout=subprocess.PIPE))#,"robotName:="+namePieces[1]

            

        
    def run(self):
        rospy.spin()
        for each in self.subprocs:
            each.terminate()

if __name__ == '__main__':
    try:
        newNode = gmapping_launcher()
        newNode.run()
    except rospy.ROSInterruptException:
        pass
