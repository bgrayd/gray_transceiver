#!/usr/bin/env python

import rospy
import socket
import threading
import json
import roslib.message
from rosbridge_library.internal import message_conversion
from Queue import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gray_transceiver.msg import GxRequest, GxMetaTopic
from rospy_message_converter import message_converter, json_message_converter


from uuid import getnode as get_mac 
MY_MAC_ADDR = get_mac()


MCAST_GRP = '224.1.1.1' #change this to use a parameter
#possibly rename this base port
META_PORT = 1025           #possibly change this to a parameter
POLLTIMERAMOUNT = 2.0 #seconds
MY_NAME = rospy.get_param("gray_transceiver/my_name", "robot")#"NOTCHANGED")
METATOPICNAME = rospy.get_param("gray_transceiver/metatopic_name","gray_transceiver/metatopic")
TOPICSIHAVE = rospy.get_param("gray_transceiver/topics_i_have",{"LIDAR":"/scan", "ODOM":"/odom"})



def recvPubSocket(sock, addr2Name, topicName, messageTypeString, metaTopic, maxsize = 65535):
    publishers = {}
    rate = rospy.Rate(10) #possible change needed
    msgTypeType = roslib.message.get_message_class(messageTypeString)
    while True:
        try:
            data2, addr = sock.recvfrom(maxsize)
            message = json.loads(data2)
            data = message_converter.convert_dictionary_to_ros_message(messageTypeString, message["data"])
            senderDomain = message["SENDER"]
        except socket.error, e:
            print 'Exception'
            continue
        
        if senderDomain in publishers:
            publishers[senderDomain].publish(data)
        else:
            newMsg = GxMetaTopic()
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
            if addr == socket.gethostbyname(socket.gethostname()):
                continue
        except socket.error, e:
            print 'Exception'
        queue.put(json.loads(data))
        rate.sleep()



class gray_transceiver(object):

    def __init__(self):
        rospy.init_node("gray_transceiver")
        
        self.debugTopic  = rospy.Publisher("Gx_DEBUG", String, queue_size = 10, latch = True)

        self.highestPortSeen = META_PORT
        self.requestQ = Queue(10)
        self.metaSockQ = Queue(20)
        self.socks = {}
        self.threadsLaunched = {}
        self.topics2PortTx = {}
        self.topics2PortRx = {}
        self.portsIUse = []
        self.names = []
        self.requested = {}
        self.timers = {}

        #this might be removed after PoC, currently unsure
        self.waitingFor = []

        #translate the IP address to the name of the sender
        self.ADDR2NAME = {}
        self.ADDR2NAME['127.0.1.1'] = "me"

        self.socks["meta"] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socks["meta"].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)
        self.socks["meta"].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socks["meta"].bind((MCAST_GRP, META_PORT))
        self.host = socket.gethostbyname(socket.gethostname())
        self.socks["meta"].setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))
        self.socks["meta"].setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(self.host))

        self.threadsLaunched["meta"] = threading.Thread(target=recvQueSocket, args=(self.socks["meta"], self.metaSockQ))
        self.threadsLaunched["meta"].daemon = True
        self.threadsLaunched["meta"].start()

        self.metaTopic = rospy.Publisher(METATOPICNAME, GxMetaTopic, queue_size = 10)
        rospy.Subscriber("gray_transceiver/requests", GxRequest, self.requests_callback)

        newMsg = {}
        newMsg["TYPE"] = "NEW"
        newMsg["SENDER"] = "NULL"
        newMsg["name requested"] = MY_NAME
        newMsg["unique id"] = str(MY_MAC_ADDR)

        self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

    #look into seeing if the message can just be put directly in the queue
    def requests_callback(self, data):
        '''
        This function gets called everytime a message is published over the /gray_transceiver/requests topic.
        '''
        temp = String()
        temp.data = "in callback"
        self.debugTopic.publish(temp)
        self.requestQ.put(data)
        
    def run(self):
        global MY_NAME
        accepted = False
        attempt = -1
        temp = String()
        temp.data = "starting"
        self.debugTopic.publish(temp)
        rate = rospy.Rate(5) #10hz probably will need to change
        noResponse = False
        for i in range(100):
            if self.metaSockQ.empty():
                noResponse = True
                rospy.sleep(0.07)#0.001)
                temp = String()
                temp.data = "in if"
                self.debugTopic.publish(temp)
            else:
                temp = String()
                temp.data = "in else"
                self.debugTopic.publish(temp)

                peek = self.metaSockQ.get()
                if peek["TYPE"] == "NEW":
                    pass
                else:
                    noResponse = False
                    self.metaSockQ.put(peek)
                    break
            temp = String()
            temp.data = "in waiting for loop"
            self.debugTopic.publish(temp)
        
        if noResponse:
            accepted = True

        while not accepted:
            message = self.metaSockQ.get()
            temp = String()
            temp.data = message
            self.debugTopic.publish(temp)
            if message["TYPE"] == "ACCEPT":
                if message["name accepted"] == MY_NAME:
                    accepted = True

                elif message["name accepted"] == MY_NAME+str(attempt):
                    accepted = True
                    MY_NAME = MY_NAME+str(attempt)
            elif message["TYPE"] == "DENY":
                if (attempt == -1) and (message["name denied"] == MY_NAME) and (message["unique id"] == MY_MAC_ADDR):
                    attempt += 1
                    newMsg = {}
                    newMsg["TYPE"] = "NEW"
                    newMsg["SENDER"] = "NULL"
                    newMsg["name requested"] = MY_NAME+str(attempy)
                    newMsg["unique id"] = str(MY_MAC_ADDR)
                    self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

                elif (message["name denied"] == MY_NAME+str(attempt)) and (message["unique id"] == MY_MAC_ADDR):
                    attempt += 1
                    newMsg = {}
                    newMsg["TYPE"] = "NEW"
                    newMsg["SENDER"] = "NULL"
                    newMsg["name requested"] = MY_NAME+str(attempy)
                    newMsg["unique id"] = str(MY_MAC_ADDR)
                    self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

            elif message["TYPE"] == "NEW":
                pass
            else:
                self.metaSockQ.put(message)
            temp = String()
            temp.data = "not yet accepted"
            self.debugTopic.publish(temp)

        self.names.append(MY_NAME)

        rospy.sleep(10)
        temp = String()
        temp.data = "accepted"
        self.debugTopic.publish(temp)
        
        while not rospy.is_shutdown():
            temp = String()
            temp.data = "in loop"
            self.debugTopic.publish(temp)
            hadMessage = False
            if not self.metaSockQ.empty():
                message = self.metaSockQ.get()

                if message["TYPE"] == "NEW":
                    temp = String()
                    temp.data = "in NEW"
                    self.debugTopic.publish(temp)
                    if message["name requested"] in self.names:
                        newMsg = {}
                        newMsg["TYPE"] = "DENY"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["name denied"] = message["name requested"]
                        newMsg["unique id"] = message["unique id"]
                        self.socks["meta"].sendto( json.dumps(newMsg), (MCAST_GRP, META_PORT))
                    else:
                        newMsg = {}
                        newMsg["TYPE"] = "ACCEPT"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["name accepted"] = message["name requested"]
                        newMsg["unique id"] = message["unique id"]
                        self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))
                        self.names.append(message["name requested"])
                        self.ADDR2NAME[message["unique id"]] = message["name requested"]

                        newMsg = {}
                        newMsg["TYPE"] = "IAM"
                        newMsg["SENDER"] = MY_NAME
                        self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

                elif message["TYPE"] == "IAM":
                    temp = String()
                    temp.data = "in IAM"
                    self.debugTopic.publish(temp)
                    if message["SENDER"] not in self.names:
                        self.names.append(message["SENDER"])

                elif message["TYPE"] == "REQUEST":
                    temp = String()
                    temp.data = "in REQUEST"
                    self.debugTopic.publish(temp)

                    if message["description"] in self.topics2PortTx:
                        newMsg = {}
                        newMsg["TYPE"] = "TXING"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] = message["description"]
                        newMsg["port"] = str(self.topics2PortTx[message["description"]])
                        newMsg["message type"] = "not yet implemented"
                        self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))


                    elif message["description"] in TOPICSIHAVE:
                        newMsg = {}
                        newMsg["TYPE"] = "IHAVE"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] =  message["description"]
                        newMsg["message type"] = TOPICSIHAVE[message["description"]]

                        self.socks["meta"].sendto(json.dumps(newMsg) ,(MCAST_GRP, META_PORT))


                elif message["TYPE"] == "SEND":
                    temp = String()
                    temp.data = "in SEND"
                    self.debugTopic.publish(temp)

                    if int(message["port"]) > self.highestPortSeen:
                            self.highestPortSeen = int(message["port"])

                    #this is where it starts call backs for sending messages
                    if message["description"] in self.topics2PortTx:
                        #do nothing cause you already are transmitting it
                        pass
                    
                    elif message["description"] in TOPICSIHAVE:
                        if message["description"] not in self.socks:
                            self.socks[message["description"]] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

                        self.host = socket.gethostbyname(socket.gethostname())
                        self.socks[message["description"]].setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))
                        self.socks[message["description"]].setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(self.host))
                        temp = String()
                        temp.data = str(MCAST_GRP)
                        self.debugTopic.publish(temp)
                        tempSock = self.socks[message["description"]]
                        tempPort = int(message["port"])
                        newMsg = {}
                        newMsg["TYPE"] = "DATA"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] = message["description"]
                        newMsg["port"] = tempPort
                        
                        def dynamicCallback(data, port=tempPort, sock = tempSock, baseMsg = newMsg):#default arguments are evaluated when the function is created, not called 
                            global MCAST_GRP
                            baseMsg["data"] = message_converter.convert_ros_message_to_dictionary(data)
                            sock.sendto(json.dumps(baseMsg), (MCAST_GRP, int(port)))

                        newMsg = {}
                        newMsg["TYPE"] = "TXING"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] = message["description"]
                        newMsg["port"] = tempPort
                        self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

                        self.topics2PortTx[message["description"]] = tempPort
                        self.portsIUse.append(int(tempPort))

                        if message["description"] == "LIDAR":
                            msgType = "sensor_msgs/LaserScan"
                        else:
                            msgType = "nav_msgs/Odometry"

                        myType = roslib.message.get_message_class(msgType)
                        rospy.Subscriber(TOPICSIHAVE[message["description"]], myType, dynamicCallback)

                elif message["TYPE"] == "TXING":
                    temp = String()
                    temp.data = "in TXING"
                    self.debugTopic.publish(temp)

                    if int(message["port"]) > self.highestPortSeen:
                            self.highestPortSeen = int(message["port"])

                    #if message["description"] is something you want, launch a thread and listen to it and publish
                    if message["description"] in self.waitingFor:
                        if message["description"] not in self.socks:
                            temp = String()
                            temp.data = "in first inner if"
                            self.debugTopic.publish(temp)
                            self.socks[message["description"]] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                            temp = String()
                            temp.data = "end of first inner if"
                            self.debugTopic.publish(temp)
                        if message["description"] not in self.threadsLaunched:
                            temp = String()
                            temp.data = "in second inner if "+MCAST_GRP+" "+str(message["port"])
                            self.debugTopic.publish(temp)
                            self.socks[message["description"]] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                            self.socks[message["description"]].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)
                            self.socks[message["description"]].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                            self.socks[message["description"]].bind((MCAST_GRP, int(message["port"])))

                            self.threadsLaunched[message["description"]] = threading.Thread(target=recvPubSocket, args=(self.socks[message["description"]], self.ADDR2NAME, message["description"], self.requested[message["description"]], self.metaTopic))
                            self.threadsLaunched[message["description"]].daemon = True
                            self.threadsLaunched[message["description"]].start()

                            self.topics2PortRx[message["description"]] = message["port"]
                            self.portsIUse.append(int(message["port"]))

                            temp = String()
                            temp.data = "end of second inner if"
                            self.debugTopic.publish(temp)
                    temp = String()
                    temp.data = "end of TXING"
                    self.debugTopic.publish(temp)

                elif message["TYPE"] == "IHAVE":
                    
                    #check against a list of messages you want but haven't heard back about yet
                    #if someone has something you want, set a timer, for a small amount of time,
                    #   after the timer goes off, if no one has said they are already txing it, 
                    #   send out a poll for the next port to use
                    #may also want the Txing to keep a dictionary of each that it has seen, with the port as the key

                    temp = String()
                    temp.data = "in IHAVE"
                    self.debugTopic.publish(temp)

                    if message["description"] in self.waitingFor:
                        portToUse = self.highestPortSeen + 1

                        newMsg = {}
                        newMsg["TYPE"] = "SEND"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] = message["description"]
                        newMsg["port"] = str(portToUse)
                        newMsg["message type"] = self.requested[message["description"]]

                        def timerCallback(sock = self.socks["meta"], message = newMsg, timersDict = self.timers, port = str(portToUse)):
                            sock.sendto(json.dumps(message), (MCAST_GRP, META_PORT))
                            timersDict[port] = None
                    
                        newMsg = {}
                        newMsg["TYPE"] = "PORT_POLL"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] = message["description"]
                        newMsg["port"] = str(portToUse)
                        newMsg["message type"] = self.requested[message["description"]]
                        self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

                        #self.portsIUse.append(portToUse)
                        self.highestPortSeen += 1

                        self.timers[str(portToUse)] = threading.Timer(POLLTIMERAMOUNT, timerCallback)
                        self.timers[str(portToUse)].start()
                
                elif message["TYPE"] == "PORT_POLL":
                    if int(message["port"]) in self.portsIUse:
                        newMsg = {}
                        newMsg["TYPE"] = "PORT_TAKEN"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] = message["description"]
                        newMsg["port"] = message["port"]
                        newMsg["message type"] = message["message type"]
                        newMsg["requestor"] = message["SENDER"]
                        self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

                elif message["TYPE"] == "PORT_TAKEN":
                    if message["requestor"] == MY_NAME:
                        if self.timers[str(message["port"])] is not None:
                            self.timers[str(message["port"])].cancel()

                            portToUse = self.highestPortSeen + 1

                        newMsg = {}
                        newMsg["TYPE"] = "SEND"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] = message["description"]
                        newMsg["port"] = str(portToUse)
                        newMsg["message type"] = self.requested[message["description"]]

                        def timerCallback(sock = self.socks["meta"], message = newMsg, timersDict = self.timers, port = str(portToUse)):
                            sock.sendto(json.dumps(message), (MCAST_GRP, META_PORT))
                            timersDict[port] = None
                            
                        newMsg = {}
                        newMsg["TYPE"] = "PORT_POLL"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] = message["description"]
                        newMsg["port"] = str(portToUse)
                        newMsg["message type"] = self.requested[message["description"]]
                        self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

                        self.portsIUse.append(portToUse)
                        # self.highestPortSeen += 1

                        self.timers[str(portToUse)] = threading.Timer(POLLTIMERAMOUNT, timerCallback)
                        self.timers[str(portToUse)].start()



            if not self.requestQ.empty():
                temp = String()
                temp.data = "got request"
                self.debugTopic.publish(temp)
                
                newRequest = self.requestQ.get()

                newMsg = {}
                newMsg["TYPE"] = "REQUEST"
                newMsg["SENDER"] = MY_NAME
                newMsg["description"] = newRequest.description
                newMsg["message type"] = newRequest.type

                self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

                #myType = getattr(__import__(str(msgTypes[0])+".msg", fromlist=[msgTypes[1]], level=1), msgTypes[1]) #put in try, can throw error if it doesn't have the attribute, or use hasattr(object, name)

                self.requested[newRequest.description] = newRequest.type#myType

                self.waitingFor.append(newRequest.description) #might be able to remove after PoC testing

            if self.metaSockQ.empty() and self.requestQ.empty():
                rate.sleep()                        # nothing to process yet, sleep longer
            else:
                rospy.sleep(0.01)                   # sleep briefly so ROS doesn't die



#consider changing meta topic to have a latch
#   or use rospy.timer and have the callback publish
if __name__ == "__main__":
    grayTransceiver = gray_transceiver()
    grayTransceiver.run()