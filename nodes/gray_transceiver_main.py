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
from gray_transceiver.msg import GxOffer, GxRequest, GxMetaTopic
from rospy_message_converter import message_converter, json_message_converter
from gray_transceiver_message import *

from uuid import getnode as get_mac 
MY_MAC_ADDR = get_mac()


MCAST_GRP = '224.1.1.1' #change this to use a parameter
#possibly rename this base port
META_PORT = 1025           #possibly change this to a parameter
MY_NAME = MY_MAC_ADDR
METATOPICNAME = rospy.get_param("gray_transceiver/metatopic_name","gray_transceiver/metatopic")
TOPICSIHAVE = rospy.get_param("gray_transceiver/topics_i_have",{"LIDAR":"/scan", "ODOM":"/odom"})

MY_IP_ADDR = subprocess.check_output(["ifconfig", "wlan0"]).split("inet addr:")[1].split(" ")[0]



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
            #TODO: consider using the message data class
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
            newMsg.myName = myName
            newMsg.name = str(senderDomain)+'/'+str(topicName)
            newMsg.type = str(messageTypeString)
            publishers[senderDomain] = rospy.Publisher(str(senderDomain)+'/'+str(topicName), msgTypeType, queue_size=10)
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
    return 1026

def portHashFromMsg(msg):
    #TODO: consider exception handaling here, such as returning a negative
    return int(portHash(msg.getDescription(), msg.getRosMsgType()))

def portHashFromRequest(request):
    #TODO: consider exception handaling here, such as returning a negative
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

        #these are lists that hold GxRequests.  If something is going wrong, such as it not recognizing that it wants a topic, check the comparision against these
        self.desired = []   #broadcast topics that you want
        self.rxing = []     #broadcast topics that you are receiving
        self.txing = []     #broacast topics that you are transmitting

        self.startedPorts = [] #list of the ports that there is something listening on

        self.socks["meta"] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socks["meta"].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 3)
        self.socks["meta"].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socks["meta"].bind((MCAST_GRP, META_PORT))
        self.host = MY_IP_ADDR#socket.gethostbyname(socket.gethostname())
        self.socks["meta"].setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))#socket.INADDR_ANY)
        self.socks["meta"].setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(self.host))
        self.socks["meta"].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)

        self.threadsLaunched["meta"] = threading.Thread(target=recvQueSocket, args=(self.socks["meta"], self.metaSockQ))
        self.threadsLaunched["meta"].daemon = True
        self.threadsLaunched["meta"].start()

        self.metaTopic = rospy.Publisher(METATOPICNAME, GxMetaTopic, queue_size = 10)
        self.availableTopic = rospy.Publisher("gray_transceiver/availableOffered", GxOffer, queue_size = 10)
        rospy.Subscriber("gray_transceiver/requests", GxRequest, self.requests_callback)
        rospy.Subscriber("gray_transceiver/offers", GxOffer, self.offers_callback)

    #look into seeing if the message can just be put directly in the queue
    def requests_callback(self, data):
        '''
        This function gets called everytime a message is published over the /gray_transceiver/requests topic.
        '''
        temp = String()
        temp.data = "in callback"
        self.debugTopic.publish(temp)
        self.requestQ.put(data)

    def offers_callback(self, data):
        '''
        This function gets called everytime a message is published over the /gray_transceiver/offers topic.
        '''
        TOPICSIHAVE[data.description] = data.type
        
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

                if message.isRequest():
                    temp = String()
                    temp.data = "in REQUEST"
                    self.debugTopic.publish(temp)

                    if message.getDescription() in self.txing:
                        newMsg = messageFactory.newTxingMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())
                        self.socks["meta"].sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))


                    elif message.getDescription() in TOPICSIHAVE:
                        newMsg = messageFactory.newIHaveMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(TOPICSIHAVE[message.getDescription()]

                        self.socks["meta"].sendto(newMsg.toJSON() ,(MCAST_GRP, META_PORT))

                elif message.isSend():
                    temp = String()
                    temp.data = "in SEND"
                    self.debugTopic.publish(temp)

                    #this is where it starts call backs for sending messages
                    if message.getDescription() in self.txing:
                        #do nothing cause you already are transmitting it
                        pass
                    
                    elif message.getDescription() in TOPICSIHAVE:
                        if message.getDescription() not in self.socks:
                            self.socks[message.getDescription()] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

                        #self.host = socket.gethostbyname(socket.gethostname())
                        self.socks[message.getDescription()].setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))#socket.INADDR_ANY)
                        self.socks[message.getDescription()].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
                        self.socks[message.getDescription()].setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(self.host))

                        tempSock = self.socks[message.getDescription()]
                        tempPort = portHashFromMsg(message)
                        newMsg = messageFactory.newDataMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())
                        
                        def dynamicCallback(data, port=tempPort, sock = tempSock, baseMsg = newMsg):#default arguments are evaluated when the function is created, not called 
                            global MCAST_GRP
                            baseMsg.setData(message_converter.convert_ros_message_to_dictionary(data))
                            sock.sendto(baseMsg.toJSON(), (MCAST_GRP, int(port)))

                        newMsg = messageFactory.newTxingMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())
                        self.socks["meta"].sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                        self.txing.append(message.getAsRequest())

                        #This will need to change, once the format of the parameters is defined
                        #TODO: I think this should be ready to be removed
                        if message["description"] == "LIDAR":
                            msgType = "sensor_msgs/LaserScan"
                        elif message["description"] == "ODOM":
                            msgType = "nav_msgs/Odometry"
                        else:
                            msgType = message.getRosMsgType()

                        myType = roslib.message.get_message_class(msgType)
                        rospy.Subscriber(TOPICSIHAVE[message.getDescription()], myType, dynamicCallback)

                elif message.isTxing():
                    temp = String()
                    temp.data = "in TXING"
                    self.debugTopic.publish(temp)

                    #if message["description"] is something you want, make sure it is listened to
                    if message.getDescription() in self.desired:

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

                            self.rxing.append(message.getAsRequest())

                        #Is the if necessary? Since the above creates it if it doesn't have it,
                        #   shouldn't it always execute?  TODO
                        if message.getDescription() in self.threadsLaunched:
                            socks_key = message.getDescription() + str(portHashFromMsg(message))
                            self.socks[socks_key] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                            self.socks[socks_key].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 3)
                            self.socks[socks_key].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                            self.socks[socks_key].bind((MCAST_GRP, portHashFromMsg(message)))

                            self.threadsLaunched[socks_key] = threading.Thread(target=recvPubSocket, args=(self.socks[message.getDescription()], message.getDescription(), message.getRosMsgType(), self.metaTopic, MY_NAME))
                            self.threadsLaunched[socks_key].daemon = True
                            self.threadsLaunched[socks_key].start()

                            self.rxing.append(message.getAsRequest())

                elif message.isIHave():

                    if message.getDescription() in self.txing:
                        newMsg = messageFactory.newTxingMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())

                        self.socks["meta"].sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                    elif message.getDescription() in self.desired:
                        newMsg = messageFactory.newSendMsg()
                        newMsg.setDescription(message.getDescription())
                        newMsg.setRosMsgType(message.getRosMsgType())

                        self.socks["meta"].sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                elif message.isOffersReq():
                    newMsg = messageFactory.newOffersAckMsg()
                    newMsg.setTopics(TOPICSIHAVE)
                    self.socks["meta"].sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                elif message.isOffersAck():
                    for key,data in message.getTopics():
                        topicOfferMsg = GxOffer() #TODO: Should this be a request instead? Since it shouldn't need the topic name?
                        topicOfferMsg.description = key
                        topicOfferMsg.type = data
                        self.availableTopic.publish(topicOfferMsg)

            if not self.requestQ.empty():
                temp = String()
                temp.data = "got request"
                self.debugTopic.publish(temp)
                
                newRequest = self.requestQ.get()

                newMsg = messageFactory.newRequestMsg()
                newMsg.setDescription(newRequest.description)
                newMsg.setRosMsgType(newRequest.type)

                self.socks["meta"].sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))

                def myTimer(timeSec = 100, msg = newMsg, sock = self.socks["meta"]):
                    import time
                    while not rospy.is_shutdown():
                        sock.sendto(newMsg.toJSON(), (MCAST_GRP, META_PORT))
                        time.sleep(timeSec)

                self.timers["request_"+newRequest.description] = threading.Thread(target=myTimer, args=())
                self.timers["request_"+newRequest.description].daemon = True
                self.timers["request_"+newRequest.description].start()

                #myType = getattr(__import__(str(msgTypes[0])+".msg", fromlist=[msgTypes[1]], level=1), msgTypes[1]) #put in try, can throw error if it doesn't have the attribute, or use hasattr(object, name)

                self.desired.append(newRequest) #need to make sure that the comparision will work right

            if self.metaSockQ.empty() and self.requestQ.empty():
                rate.sleep()                        # nothing to process yet, sleep longer
            else:
                rospy.sleep(0.01)                   # sleep briefly so ROS doesn't die



#consider changing meta topic to have a latch
#   or use rospy.timer and have the callback publish
if __name__ == "__main__":
    grayTransceiver = gray_transceiver()
    grayTransceiver.run()