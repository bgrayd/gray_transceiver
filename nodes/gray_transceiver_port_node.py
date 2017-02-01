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

METATOPICNAME = rospy.get_param("/gray_transceiver/metatopic_name","/gray_transceiver/metatopic")

json = rospy.get_param("throughput_test/json_bool")


class gray_transceiver_port(object):
    
    def __init__(self):
        rospy.init_node("port")
        self.broadcastTopicsToReceive = []
        self.broadcastTopicsToTransmit = []
        self.receiveStarted = False
        self.transmitStarted = False

        self.mcast_group = rospy.get_param("/gray_transceiver/multicast_group", '224.1.1.1')
        self.myIp = rospy.get_param("/gray_transceiver/ip_to_use", '127.0.0.1')
        self.myName = rospy.get_param("/gray_transceiver/my_name", "INVALID")
        self.port = int(sys.argv[1])

        self.messageFactory = GxMessageFactory(name = self.myName)
        
        nodeName = str(rospy.get_name())

        self.requestService = rospy.Service(nodeName+"/receive", GxRequest, self.receive_callback)
        self.offerService = rospy.Service(nodeName+"/transmit", GxOffer, self.transmit_callback)

        self.metaTopic = rospy.Publisher(METATOPICNAME, GxMetaTopic, queue_size = 10)

    def receive_callback(self, data):
        if str(data.topicMetaInfo) not in self.broadcastTopicsToReceive:
            self.broadcastTopicsToReceive.append(str(data.topicMetaInfo))

        if not self.receiveStarted:
            self.receiveSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            self.receiveSocket.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.myIp))
            self.receiveSocket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
            self.receiveSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.receiveSocket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 3)
            self.receiveSocket.bind((self.mcast_group, self.port))
        
        self.receiveStarted = True

        return GxRequestResponse(True)

    def transmit_callback(self, data):
        if str(data.topicMetaInfo) in self.broadcastTopicsToTransmit:
            return GxOfferResponse(True)

        self.broadcastTopicsToTransmit.append(str(data.topicMetaInfo))

        if not self.transmitStarted:
            self.transmitSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            self.transmitSocket.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.myIp))
            self.transmitSocket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
            self.transmitSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.transmitSocket.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(self.mcast_group) + socket.inet_aton(self.myIp))


        self.transmitStarted = True

        newMsg = self.messageFactory.newDataMsg()
        newMsg.setDescription(data.topicMetaInfo.description)
        newMsg.setRosMsgType(data.topicMetaInfo.type)

        #set up the callback for the local topic that will be transmitted                        
        def dynamicCallback(data, port=self.port, sock = self.transmitSocket, baseMsg = newMsg):#default arguments are evaluated when the function is created, not called 
            baseMsg.setDataFromRosMsg(data)
            if json:
                socketData = baseMsg.toJSON()
            else:
                socketData = baseMsg.toBitString()
            sock.sendto(socketData, (self.mcast_group, int(port)))

        myType = roslib.message.get_message_class(data.topicMetaInfo.type)
        rospy.Subscriber(data.topicName, myType, dynamicCallback)

        return GxOfferResponse(True)

    def run(self):
        publishers = {}
        startRate = rospy.Rate(10)
        runningRate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.receiveStarted:
                newData = GxDataMsg()
                senderDomain = ""
                try:
                    data2, addr = self.receiveSocket.recvfrom(65535)
                    if json:
                        newData.fromJSON()
                    else:
                        newData.fromBitString()
                    #newData.fromSocket(data2)
                    senderDomain = str(newData.getSender()) + str(newData.getTopicMetaInformation())
                except socket.error, e:
                    print 'Exception'
                    continue

                if str(newData.getTopicMetaInformation()) not in self.broadcastTopicsToReceive:
                    continue
                
                if senderDomain not in publishers:
                    newMsg = GxMetaTopic()
                    newMsg.myName = str(self.myName)
                    newMsg.name = '/foreign_'+str(newData.getSender())+'/'+str(newData.getDescription())
                    newMsg.type = str(newData.getRosMsgType())
                    msgTypeType = roslib.message.get_message_class(newData.getRosMsgType())
                    publishers[senderDomain] = rospy.Publisher('/foreign_'+str(newData.getSender())+'/'+str(newData.getDescription()), msgTypeType, queue_size=10)
                    self.metaTopic.publish(newMsg)
                publishers[senderDomain].publish(newData.getDataAsRosMsg())
                runningRate.sleep()
            else:
                startRate.sleep()


if __name__=="__main__":
    if len(sys.argv) < 2:
        print("This should only be called by the gray transceiver internally and the wrong number of arguments were given")
    else:
        portNode = gray_transceiver_port()
        portNode.run()