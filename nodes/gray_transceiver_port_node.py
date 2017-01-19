#!/usr/bin/env python

import rospy
import socket
import threading
import json
import subprocess
import sys
import roslib.message
from Queue import *
from std_msgs.msg import String
from gray_transceiver.msg import GxTopicMetaInformation, GxMetaTopic
from gray_transceiver.srv import GxOffer, GxRequest, GxOfferResponse, GxRequestResponse
from rospy_message_converter import message_converter, json_message_converter
from gray_transceiver_message import *

#TODO: remove this comment block once completed.
#import roslaunch
#
#in __init__ add following lines
#self.launch = roslaunch.scriptapi.ROSLaunch()
#self.launch.start()
#self.portNodes = []
#rospy.on_shutdown(self.killPorts)
#
#
#add following function:
#def killPorts(self):
#   for each in self.portNodes:
#       each.stop()
#   self.portNodes = []
#
#
#to start new port:
#newPort = roslaunch.core.Node(package, executable,name=nodeName, args=arguments)
#self.portNodes.append(self.launch.launch(self.newPort))

PORT = None

class gray_transceiver_port(object):
    def __init__(self):
        rospy.init_node("port")

if __name__=="__main__":
    if len(sys.argv) != 2:
        print("This should only be called by the gray transceiver internally and the wrong number of arguments were given")
    else:
        global PORT
        PORT = sys.argv[1]