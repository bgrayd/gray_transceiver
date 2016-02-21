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
            namePieces = data.name.split("/")#"scan:="+data.name, "map_metadata:="+namePieces[0]+"/map_metadata", "map:="+namePieces[0]+"/map",
            rospy.set_param(namePieces[0]+"/robotName", namePieces[0])
            self.subprocs.append(subprocess.Popen(["rosrun","gmapping", "slam_gmapping", "__ns:="+namePieces[0], "scan:=reframed_scan","_map_frame:="+namePieces[0]+"_map", "_base_frame:="+namePieces[0]+"_base_link", "_odom_frame:="+namePieces[0]+"_odom" ], stdin=subprocess.PIPE, stdout=subprocess.PIPE))
            self.subprocs.append(subprocess.Popen(["rosrun","gray_transceiver", "lidar_remapper.py", "__ns:="+namePieces[0]], stdin=subprocess.PIPE, stdout=subprocess.PIPE))#,"robotName:="+namePieces[0]

            

        
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
