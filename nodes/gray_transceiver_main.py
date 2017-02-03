#!/usr/bin/env python

import rospy
import socket
import threading
import json
import subprocess
import roslib.message
import roslaunch
# from rosbridge_library.internal import message_conversion
from Queue import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gray_transceiver.msg import GxTopicMetaInformation, GxMetaTopic
from gray_transceiver.srv import GxOffer, GxRequest, GxOfferResponse, GxRequestResponse #TODO: consider replacing with import *
from rospy_message_converter import message_converter, json_message_converter
from gray_transceiver_message import *

from uuid import getnode as get_mac 
MY_MAC_ADDR = get_mac()


MCAST_GRP = '224.1.1.1' #change this to use a parameter
META_PORT = 1025        #possibly change this to a parameter
MY_NAME = str(MY_MAC_ADDR)
METATOPICNAME = rospy.get_param("gray_transceiver/metatopic_name","/gray_transceiver/metatopic")
TOPICSIHAVE = rospy.get_param("gray_transceiver/topics_i_have",{"LIDAR":"/scan", "ODOM":"/odom"})
INTERFACE_TO_USE = rospy.get_param("gray_transceiver/interface_to_use","wlan0") #TODO: testing changed wlan0 to lo
MY_IP_ADDR = subprocess.check_output(["ifconfig", INTERFACE_TO_USE]).split("inet addr:")[1].split(" ")[0]

rospy.set_param("/gray_transceiver/multicast_group", MCAST_GRP)
rospy.set_param("/gray_transceiver/ip_to_use", MY_IP_ADDR)
rospy.set_param("/gray_transceiver/my_name", MY_NAME)
rospy.set_param("/gray_transceiver/metatopic_name", METATOPICNAME)


def recvQueSocket(sock, queue, maxsize = 1024):
    rate = rospy.Rate(10) #possibly change
    while True:
        try:
            data, addr = sock.recvfrom(maxsize)
        except socket.error, e:
            print 'Exception'
            continue

        queue.put(data)
        rate.sleep()

def hashString(string):
    h = 0
    for each in string:
        h = (h << 11) ^ (h >>3) ^ ord(each)
    return h

#TODO: consider returning a negative on errors, so the Gx will keep running and not crash. 
def portHash(description=None, rosMsgType=None):
    hash1 = hashString(description)
    hash2 = hashString(rosMsgType)
    hash3 = hash1 ^ hash2
    hash4 = hash3 % 65535
    if hash4 <= META_PORT:
        return hash4 + META_PORT
    return hash4 

def portHashFromMsg(msg):
    return int(portHash(msg.getDescription(), msg.getRosMsgType()))

def portHashFromTopicMetaInfo(request):
    return int(portHash(request.description, request.type))

class gray_transceiver(object):

    def __init__(self):
        rospy.init_node("gray_transceiver")
        
        self.debugTopic  = rospy.Publisher("Gx_DEBUG", String, queue_size = 10, latch = True)

        self.metaSockQ = Queue(20)
        self.threadsLaunched = {}
        self.timers = {}
        self.offersAvailable = {}

        #these are lists that hold GxTopicMetaInformations.  If something is going wrong, such as it not recognizing that it wants a topic, check the comparision against these
        self.desired = []   #broadcast topics (as strings) that you want
        self.rxing = []     #broadcast topics (as strings) that you are receiving
        self.txing = []     #broacast topics (as strings) that you are transmitting

        self.startedPorts = [] #list of the ports that there is something listening on


        self.messageFactory = GxMessageFactory(name = MY_NAME)

        self.metaSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.metaSocket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 3)
        self.metaSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.metaSocket.bind((MCAST_GRP, META_PORT))
        self.host = MY_IP_ADDR#socket.gethostbyname(socket.gethostname())
        self.metaSocket.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))#socket.INADDR_ANY)
        self.metaSocket.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(self.host))
        self.metaSocket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)

        self.threadsLaunched["meta"] = threading.Thread(target=recvQueSocket, args=(self.metaSocket, self.metaSockQ))
        self.threadsLaunched["meta"].daemon = True
        self.threadsLaunched["meta"].start()

        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        self.portNodes = []
        rospy.on_shutdown(self.killPorts)

        self.metaTopic = rospy.Publisher(METATOPICNAME, GxMetaTopic, queue_size = 10)
        self.availableTopic = rospy.Publisher("gray_transceiver/availableOffered", GxTopicMetaInformation, queue_size = 10)
        self.requestService = rospy.Service("gray_transceiver/requests", GxRequest, self.requests_callback)
        self.offerService = rospy.Service("gray_transceiver/offers", GxOffer, self.offers_callback)

    def killPorts(self):
      for each in self.portNodes:
          each.stop()
      self.portNodes = []

    def setUpPort(self, topicMetaInfo):
        newPortNumber = portHashFromTopicMetaInfo(topicMetaInfo)
        if newPortNumber not in self.startedPorts:
            self.startedPorts.append(newPortNumber)

            package = "gray_transceiver"
            executable = "gray_transceiver_port_node.py"
            nodeNamespace = "/gray_transceiver/"
            nodeName = "port"+str(newPortNumber)

            arguments = ""
            arguments += str(newPortNumber)

            newPort = roslaunch.core.Node(package, executable,name=nodeName, namespace=nodeNamespace, args=arguments)
            self.portNodes.append(self.launch.launch(newPort))

        return newPortNumber

    def startTransmitting(self, broadcastTopic):
        if str(broadcastTopic) in self.txing:
            return
        newPortNumber = self.setUpPort(broadcastTopic)

        rospy.wait_for_service("/gray_transceiver/port"+str(newPortNumber)+"/transmit")

        transmitRequest = rospy.ServiceProxy("/gray_transceiver/port"+str(newPortNumber)+"/transmit", GxOffer)
        transmitRequest(broadcastTopic, self.offersAvailable[str(broadcastTopic)]["topicName"])
        self.txing.append(str(broadcastTopic))
    
    def startReceiving(self, broadcastTopic):
        if str(broadcastTopic) in self.rxing:
            return
        newPortNumber = self.setUpPort(broadcastTopic)
        rospy.wait_for_service("/gray_transceiver/port"+str(newPortNumber)+"/receive")
        transmitRequest = rospy.ServiceProxy("/gray_transceiver/port"+str(newPortNumber)+"/receive", GxRequest)
        transmitRequest(broadcastTopic)
        self.rxing.append(str(broadcastTopic))  

    def requests_callback(self, data):
        '''
        This function gets called everytime someone wants to use the request service
        '''
        newRequest = data.topicMetaInfo

        #Send a request for the broadcast topic, and set up a timer to continue occasionally asking for it
        newMsg = self.messageFactory.newSendMsg()
        newMsg.setDescription(newRequest.description)
        newMsg.setRosMsgType(newRequest.type)

        def myTimer(timeSec = 100, msg = newMsg, sock = self.metaSocket):
            import time
            while not rospy.is_shutdown():
                sock.sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))
                time.sleep(timeSec)

        self.timers["request_"+newRequest.description] = threading.Thread(target=myTimer, args=())
        self.timers["request_"+newRequest.description].daemon = True
        self.timers["request_"+newRequest.description].start()

        self.desired.append(str(newRequest))

        return GxRequestResponse(True)

    def offers_callback(self, data):
        '''
        This function gets called everytime a message is published over the /gray_transceiver/offers topic.
        '''
        self.offersAvailable[str(data.topicMetaInfo)] = {"topicMetaInfo":data.topicMetaInfo, "topicName":data.topicName}
        return GxOfferResponse(True)
        
    def run(self):
        global MY_NAME

        temp = String()
        temp.data = "starting"
        self.debugTopic.publish(temp)

        rate = rospy.Rate(10) #10hz probably will need to change
        
        while not rospy.is_shutdown():
            if not self.metaSockQ.empty():
                message = self.messageFactory.fromJSON(self.metaSockQ.get())

                #Someone is requesting a broadcast topic
                if message.isRequest():
                    temp = String()
                    temp.data = "in REQUEST"
                    self.debugTopic.publish(temp)

                    #if your are transmitting it, tell them you are
                    if str(message.getTopicMetaInformation()) in self.txing:
                        newMsg = self.messageFactory.newTxingMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())
                        self.metaSocket.sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                    #otherwise, tell them if you have it
                    elif str(message.getTopicMetaInformation()) in self.offersAvailable:
                        newMsg = self.messageFactory.newIHaveMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())
                        self.metaSocket.sendto(newMsg.toJSON() ,(MCAST_GRP, META_PORT))

                #Someone is asking a broadcast topic to be sent
                elif message.isSend():
                    temp = String()
                    temp.data = "in SEND"
                    self.debugTopic.publish(temp)

                    #It is already being transmitted, do nothing
                    if str(message.getTopicMetaInformation()) in self.txing:
                        newMsg = self.messageFactory.newTxingMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())
                        self.metaSocket.sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))
                    
                    #otherwise, if you have the topic, start sending it
                    elif str(message.getTopicMetaInformation()) in self.offersAvailable:
                        self.startTransmitting(message.getTopicMetaInformation())

                        newMsg = self.messageFactory.newTxingMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())
                        self.metaSocket.sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                #Someone is saying that they are transmitting a broadcast topic
                elif message.isTxing():
                    temp = String()
                    temp.data = "in TXING"
                    self.debugTopic.publish(temp)

                    #if message["description"] is something you want, make sure it is listened to
                    if str(message.getTopicMetaInformation()) in self.desired:
                        self.startReceiving(message.getTopicMetaInformation())

                #Someone is saying that they have a broadcast topic
                elif message.isIHave():

                    #if it is something you want, tell them to send it
                    if str(message.getTopicMetaInformation()) in self.desired:
                        newMsg = self.messageFactory.newSendMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())

                        self.metaSocket.sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                    #otherwise, if the broadcast topic is one you are transmitting, say that it is being transmitted
                    elif str(message.getTopicMetaInformation()) in self.txing:
                        newMsg = self.messageFactory.newTxingMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())

                        self.metaSocket.sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                #Someone asked what broadcast topics are available
                elif message.isOffersReq():
                    newMsg = self.messageFactory.newOffersAckMsg()
                    newMsg.setTopics(self.offersAvailable.keys())
                    self.metaSocket.sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                #A response to a request of what topics are available
                elif message.isOffersAck():
                    for key,data in message.getTopics():
                        topicOfferMsg = GxTopicMetaInformation()
                        topicOfferMsg.description = key
                        topicOfferMsg.type = data
                        self.availableTopic.publish(topicOfferMsg)

            if self.metaSockQ.empty():
                rate.sleep()                        # nothing to process yet, sleep longer
            else:
                rospy.sleep(0.01)                   # sleep briefly so ROS doesn't die



#consider changing meta topic to have a latch
#   or use rospy.timer and have the callback publish
if __name__ == "__main__":
    grayTransceiver = gray_transceiver()
    grayTransceiver.run()