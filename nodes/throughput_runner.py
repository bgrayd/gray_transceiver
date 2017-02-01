#!/usr/bin/env python

import rospy
import StringIO, struct
from std_msgs.msg import String
from gray_transceiver.msg import GxTopicMetaInformation, throughputTest
from gray_transceiver.srv import GxRequest, GxOffer


class runSettings(object):
    

class run(object):
    def __init__(self, runSettings):


class client(object):

    def __init__(self):
        '''
        Constructor for client class.
        '''
        rospy.init_node("client")
        
    def run(self):
        '''
        Do the initial requests and then do nothing
        '''


if __name__ == "__main__":
    Client = client()
    Client.run()