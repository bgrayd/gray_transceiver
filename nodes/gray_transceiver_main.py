#!/usr/bin/env python

import rospy
import socket
import threading
import json
import subprocess
import roslib.message
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


MCAST_GRP = '224.1.1.1' #change this to use a parameter   #TODO: testing changed '224.1.1.1' to ''
META_PORT = 1025        #possibly change this to a parameter
MY_NAME = str(MY_MAC_ADDR)
METATOPICNAME = rospy.get_param("gray_transceiver/metatopic_name","gray_transceiver/metatopic")
TOPICSIHAVE = rospy.get_param("gray_transceiver/topics_i_have",{"LIDAR":"/scan", "ODOM":"/odom"})

MY_IP_ADDR = subprocess.check_output(["ifconfig", "lo"]).split("inet addr:")[1].split(" ")[0] #TODO: testing changed wlan0 to lo



#TODO: change so it checks the topic meta information, so it needs to use the message class
def recvPubSocket(sock, topicName, messageTypeString, metaTopic,myName, maxsize = 65535):
    publishers = {}
    rate = rospy.Rate(10) #possible change needed
    msgTypeType = roslib.message.get_message_class(messageTypeString)
    temp = String()
    temp.data = "new recvPubSocket"+str(topicName)
    rospy.Publisher("Gx_DEBUG", String, queue_size = 10, latch = True).publish(temp)
    while True:
        try:
            data2, addr = sock.recvfrom(maxsize)
            #TODO: use the message data class
            message = json.loads(data2)
            try:
                data = message_converter.convert_dictionary_to_ros_message(messageTypeString, message["data"])
            except Exception as ex:
                data = message_converter.convert_dictionary_to_ros_message(messageTypeString, json.loads(message["data"]))
            senderDomain = message["SENDER"]
        except socket.error, e:
            print 'Exception'
            continue
        
        if senderDomain in publishers:
            publishers[senderDomain].publish(data)
        else:
            newMsg = GxMetaTopic()
            newMsg.myName = str(myName)
            newMsg.name = 'foreign/'+str(senderDomain)+'/'+str(topicName)
            newMsg.type = str(messageTypeString)
            publishers[senderDomain] = rospy.Publisher('foreign/'+str(senderDomain)+'/'+str(topicName), msgTypeType, queue_size=10)
            metaTopic.publish(newMsg)
            publishers[senderDomain].publish(data)
        rate.sleep()

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

def portHash(description=None, rosMsgType=None):
    #exceptions should return a negative number
    return 1026

def portHashFromMsg(msg):
    #TODO: exceptions should return a negative number
    return int(portHash(msg.getDescription(), msg.getRosMsgType()))

def portHashFromTopicMetaInfo(request):
    #TODO: exceptions should return a negative number
    return int(portHash(request.description(), request.type()))

class gray_transceiver(object):

    def __init__(self):
        rospy.init_node("gray_transceiver")
        
        self.debugTopic  = rospy.Publisher("Gx_DEBUG", String, queue_size = 10, latch = True)

        self.requestQ = Queue(10)
        self.metaSockQ = Queue(20)
        self.socks = {}
        self.threadsLaunched = {}
        self.timers = {}
        self.offersAvailable = {}

        #these are lists that hold GxTopicMetaInformations.  If something is going wrong, such as it not recognizing that it wants a topic, check the comparision against these
        self.desired = []   #broadcast topics (as strings) that you want
        self.rxing = []     #broadcast topics (as strings) that you are receiving
        self.txing = []     #broacast topics (as strings) that you are transmitting

        self.startedPorts = [] #list of the ports that there is something listening on

        self.socks["meta"] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socks["meta"].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 3)
        self.socks["meta"].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socks["meta"].bind(('', META_PORT)) #TODO: testing changed MCAST_GRP to ''
        self.host = MY_IP_ADDR#socket.gethostbyname(socket.gethostname())
        self.socks["meta"].setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))#socket.INADDR_ANY)
        self.socks["meta"].setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(self.host))
        self.socks["meta"].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)

        self.threadsLaunched["meta"] = threading.Thread(target=recvQueSocket, args=(self.socks["meta"], self.metaSockQ))
        self.threadsLaunched["meta"].daemon = True
        self.threadsLaunched["meta"].start()

        self.metaTopic = rospy.Publisher(METATOPICNAME, GxMetaTopic, queue_size = 10)
        self.availableTopic = rospy.Publisher("gray_transceiver/availableOffered", GxTopicMetaInformation, queue_size = 10)
        self.requestService = rospy.Service("gray_transceiver/requests", GxRequest, self.requests_callback)
        self.offerService = rospy.Service("gray_transceiver/offers", GxOffer, self.offers_callback)

    def requests_callback(self, data):
        '''
        This function gets called everytime someone wants to use the request service
        '''
        temp = String()
        temp.data = "in requests callback"
        self.debugTopic.publish(temp)
        self.requestQ.put(data.topicMetaInfo)
        return GxRequestResponse(True)

    def offers_callback(self, data):
        '''
        This function gets called everytime a message is published over the /gray_transceiver/offers topic.
        '''
        self.offersAvailable[str(data.topicMetaInfo)] = {"topicMetaInfo":data.topicMetaInfo, "topicName":data.topicName}
        print(self.offersAvailable)
        return GxOfferResponse(True)
        
    def run(self):
        global MY_NAME

        temp = String()
        temp.data = "starting"
        self.debugTopic.publish(temp)

        rate = rospy.Rate(10) #10hz probably will need to change
        
        messageFactory = GxMessageFactory(name = MY_NAME)
        
        while not rospy.is_shutdown():
            if not self.metaSockQ.empty():
                message = messageFactory.fromJSON(self.metaSockQ.get())

                #Someone is requesting a broadcast topic
                if message.isRequest():
                    temp = String()
                    temp.data = "in REQUEST"
                    self.debugTopic.publish(temp)

                    #if your are transmitting it, tell them you are
                    if str(message.getTopicMetaInformation()) in self.txing:
                        newMsg = messageFactory.newTxingMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())
                        self.socks["meta"].sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                    #otherwise, tell them if you have it
                    elif str(message.getTopicMetaInformation()) in self.offersAvailable:
                        newMsg = messageFactory.newIHaveMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())
                        self.socks["meta"].sendto(newMsg.toJSON() ,(MCAST_GRP, META_PORT))

                #Someone is asking a broadcast topic to be sent
                elif message.isSend():
                    temp = String()
                    temp.data = "in SEND"
                    self.debugTopic.publish(temp)

                    #It is already being transmitted, do nothing
                    if str(message.getTopicMetaInformation()) in self.txing:
                        temp.data = "thinks it is already sending"
                        self.debugTopic.publish(temp)
                    
                    #otherwise, if you have the topic, start sending it
                    elif str(message.getTopicMetaInformation()) in self.offersAvailable:
                        if message.getDescription() not in self.socks:
                            self.socks[message.getDescription()] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

                        self.socks[message.getDescription()].setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))#socket.INADDR_ANY)
                        self.socks[message.getDescription()].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
                        self.socks[message.getDescription()].setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(self.host))

                        tempSock = self.socks[message.getDescription()]
                        tempPort = portHashFromMsg(message)
                        newMsg = messageFactory.newDataMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())

                        #set up the callback for the local topic that will be transmitted                        
                        def dynamicCallback(data, port=tempPort, sock = tempSock, baseMsg = newMsg):#default arguments are evaluated when the function is created, not called 
                            global MCAST_GRP
                            baseMsg.setData(message_converter.convert_ros_message_to_dictionary(data))
                            sock.sendto(baseMsg.toJSON(), (MCAST_GRP, int(port)))

                        newMsg = messageFactory.newTxingMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())
                        self.socks["meta"].sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                        self.txing.append(str(message.getTopicMetaInformation()))

                        msgType = message.getRosMsgType()

                        myType = roslib.message.get_message_class(msgType)
                        rospy.Subscriber(self.offersAvailable[str(message.getTopicMetaInformation())]["topicName"], myType, dynamicCallback)

                #Someone is saying that they are transmitting a broadcast topic
                elif message.isTxing():
                    temp = String()
                    temp.data = "in TXING"
                    self.debugTopic.publish(temp)

                    #if message["description"] is something you want, make sure it is listened to
                    if str(message.getTopicMetaInformation()) in self.desired:

                        #This is insanity. I think I had stuff in the wrong order or something, idk
                        #  but this needs to go.  Instead, have something like:
                        #   if message is desired:
                        #       if message not in rxing:
                        #           check port, if not being used, start it and listen
                        if message.getDescription() not in self.socks:
                            self.socks[message.getDescription()] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                        if message.getDescription() not in self.threadsLaunched:
                            self.socks[message.getDescription()] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                            self.socks[message.getDescription()].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 3)
                            self.socks[message.getDescription()].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                            self.socks[message.getDescription()].bind((MCAST_GRP, portHashFromMsg(message)))

                            self.threadsLaunched[message.getDescription()] = threading.Thread(target=recvPubSocket, args=(self.socks[message.getDescription()], message.getDescription(), message.getRosMsgType(), self.metaTopic, MY_NAME))
                            self.threadsLaunched[message.getDescription()].daemon = True
                            self.threadsLaunched[message.getDescription()].start()

                            self.rxing.append(str(message.getTopicMetaInformation()))

                        #TODO remove this like above
                        if message.getDescription() in self.threadsLaunched:
                            socks_key = message.getDescription() + str(portHashFromMsg(message))
                            self.socks[socks_key] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                            self.socks[socks_key].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 3)
                            self.socks[socks_key].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                            self.socks[socks_key].bind((MCAST_GRP, portHashFromMsg(message)))

                            self.threadsLaunched[socks_key] = threading.Thread(target=recvPubSocket, args=(self.socks[message.getDescription()], message.getDescription(), message.getRosMsgType(), self.metaTopic, MY_NAME))
                            self.threadsLaunched[socks_key].daemon = True
                            self.threadsLaunched[socks_key].start()

                            self.rxing.append(str(message.getTopicMetaInformation()))

                #Someone is saying that they have a broadcast topic
                elif message.isIHave():

                    #if the broadcast topic is one you are transmitting, say that it is being transmitted
                    if str(message.getTopicMetaInformation()) in self.txing:
                        newMsg = messageFactory.newTxingMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())

                        self.socks["meta"].sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                    #otherwise, if it is something you want, tell them to send it
                    elif str(message.getTopicMetaInformation()) in self.desired:
                        newMsg = messageFactory.newSendMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())

                        self.socks["meta"].sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                #Someone asked what broadcast topics are available
                elif message.isOffersReq():
                    newMsg = messageFactory.newOffersAckMsg()
                    newMsg.setTopics(self.offersAvailable.keys())
                    self.socks["meta"].sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                #A response to a request of what topics are available
                elif message.isOffersAck():
                    for key,data in message.getTopics():
                        topicOfferMsg = GxTopicMetaInformation()
                        topicOfferMsg.description = key
                        topicOfferMsg.type = data
                        self.availableTopic.publish(topicOfferMsg)

            #check the request queue to see if there are any requests for new broadcast topics
            #TODO: move all of this into the service
            if not self.requestQ.empty():
                temp = String()
                temp.data = "got request"
                self.debugTopic.publish(temp)
                
                newRequest = self.requestQ.get()

                #Send a request for the broadcast topic, and set up a timer to continue occasionally asking for it
                newMsg = messageFactory.newRequestMsg()
                newMsg.setDescription(newRequest.description)
                newMsg.setRosMsgType(newRequest.type)

                def myTimer(timeSec = 100, msg = newMsg, sock = self.socks["meta"]):
                    import time
                    while not rospy.is_shutdown():
                        sock.sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))
                        time.sleep(timeSec)

                self.timers["request_"+newRequest.description] = threading.Thread(target=myTimer, args=())
                self.timers["request_"+newRequest.description].daemon = True
                self.timers["request_"+newRequest.description].start()

                #myType = getattr(__import__(str(msgTypes[0])+".msg", fromlist=[msgTypes[1]], level=1), msgTypes[1]) #put in try, can throw error if it doesn't have the attribute, or use hasattr(object, name)

                self.desired.append(str(newRequest)) #need to make sure that the comparision will work right

            if self.metaSockQ.empty() and self.requestQ.empty():
                rate.sleep()                        # nothing to process yet, sleep longer
            else:
                rospy.sleep(0.01)                   # sleep briefly so ROS doesn't die



#consider changing meta topic to have a latch
#   or use rospy.timer and have the callback publish
if __name__ == "__main__":
    grayTransceiver = gray_transceiver()
    grayTransceiver.run()