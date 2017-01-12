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
POLLTIMERAMOUNT = 2.5 #seconds
MY_NAME = rospy.get_param("gray_transceiver/my_name", "robot")#"NOTCHANGED")
METATOPICNAME = rospy.get_param("gray_transceiver/metatopic_name","gray_transceiver/metatopic")
TOPICSIHAVE = rospy.get_param("gray_transceiver/topics_i_have",{"LIDAR":"/scan", "ODOM":"/odom"})

MY_IP_ADDR = subprocess.check_output(["ifconfig", "wlan0"]).split("inet addr:")[1].split(" ")[0]



def recvPubSocket(sock, addr2Name, topicName, messageTypeString, metaTopic,myName, maxsize = 65535):
    publishers = {}
    rate = rospy.Rate(10) #possible change needed
    msgTypeType = roslib.message.get_message_class(messageTypeString)
    temp = String()
    temp.data = "new recvPubSocket"+str(topicName)
    rospy.Publisher("Gx_DEBUG", String, queue_size = 10, latch = True).publish(temp)
    while True:
        try:
            data2, addr = sock.recvfrom(maxsize)
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

        newMsg = {}
        newMsg["TYPE"] = "NEW"
        newMsg["SENDER"] = "NULL"
        newMsg["name_requested"] = MY_NAME
        newMsg["unique_id"] = str(MY_MAC_ADDR)

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

    def offers_callback(self, data):
        '''
        This function gets called everytime a message is published over the /gray_transceiver/offers topic.
        '''
        TOPICSIHAVE[data.description] = data.type
        
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
                noResponse = noResponse or True
                rospy.sleep(0.001)#0.9)#0.001)
                # temp = String()
                # temp.data = "in if"
                # self.debugTopic.publish(temp)
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

        iterationsSinceResponse = 0
        while not accepted:
            if self.metaSockQ.empty():
                rospy.sleep(0.1)
                iterationsSinceResponse += 1
                if iterationsSinceResponse >= 100:
                    iterationsSinceResponse = 0
                    newMsg = {}
                    newMsg["TYPE"] = "NEW"
                    newMsg["SENDER"] = "NULL"
                    newMsg["name_requested"] = MY_NAME+str(attempt)
                    newMsg["unique_id"] = str(MY_MAC_ADDR)
                    self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))
                continue
            message = self.metaSockQ.get()
            if message["TYPE"] == "ACCEPT":
                if message["name_requested"] == MY_NAME:
                    accepted = True

                elif message["name_requested"] == MY_NAME+str(attempt):
                    accepted = True
                    MY_NAME = MY_NAME+str(attempt)
            elif message["TYPE"] == "DENY":
                if (attempt == -1) and (message["name_requested"] == MY_NAME) and (message["unique_id"] == str(MY_MAC_ADDR)):
                    attempt += 1
                    newMsg = {}
                    newMsg["TYPE"] = "NEW"
                    newMsg["SENDER"] = "NULL"
                    newMsg["name_requested"] = MY_NAME+str(attempt)
                    newMsg["unique_id"] = str(MY_MAC_ADDR)
                    self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

                elif (message["name_requested"] == MY_NAME+str(attempt)) and (message["unique_id"] == str(MY_MAC_ADDR)):
                    attempt += 1
                    newMsg = {}
                    newMsg["TYPE"] = "NEW"
                    newMsg["SENDER"] = "NULL"
                    newMsg["name_requested"] = MY_NAME+str(attempt)
                    newMsg["unique_id"] = str(MY_MAC_ADDR)
                    self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

            elif message["TYPE"] == "NEW":
                pass
            else:
                #self.metaSockQ.put(message)
                pass
            temp = String()
            temp.data = "not yet accepted"
            self.debugTopic.publish(temp)

        self.names.append(MY_NAME)

        rospy.sleep(10)
        temp = String()
        temp.data = "accepted as "+MY_NAME
        self.debugTopic.publish(temp)
        
        while not rospy.is_shutdown():
            # temp = String()
            # temp.data = "in loop"
            # self.debugTopic.publish(temp)
            hadMessage = False
            if not self.metaSockQ.empty():
                message = self.metaSockQ.get()

                if message["TYPE"] == "NEW":
                    temp = String()
                    temp.data = "in NEW"
                    self.debugTopic.publish(temp)
                    if message["name_requested"] in self.names:
                        newMsg = {}
                        newMsg["TYPE"] = "DENY"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["name_requested"] = message["name_requested"]
                        newMsg["unique_id"] = message["unique_id"]
                        self.socks["meta"].sendto( json.dumps(newMsg), (MCAST_GRP, META_PORT))
                    else:
                        newMsg = {}
                        newMsg["TYPE"] = "ACCEPT"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["name_requested"] = message["name_requested"]
                        newMsg["unique_id"] = message["unique_id"]
                        self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))
                        self.names.append(message["name_requested"])
                        self.ADDR2NAME[message["unique_id"]] = message["name_requested"]

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
                        newMsg["port"] = int(self.topics2PortTx[message["description"]])
                        newMsg["message_type"] = "not yet implemented"
                        self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

                    # elif message["description"] in self.topics2PortRx:
                    #     newMsg = {}
                    #     newMsg["TYPE"] = "TXING"
                    #     newMsg["SENDER"] = MY_NAME
                    #     newMsg["description"] = message["description"]
                    #     newMsg["port"] = str(self.topics2PortRx[message["description"]])
                    #     newMsg["message_type"] = "not yet implemented"
                    #     self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))


                    elif message["description"] in TOPICSIHAVE:
                        newMsg = {}
                        newMsg["TYPE"] = "IHAVE"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] =  message["description"]
                        newMsg["message_type"] = TOPICSIHAVE[message["description"]]

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

                        #self.host = socket.gethostbyname(socket.gethostname())
                        self.socks[message["description"]].setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))#socket.INADDR_ANY)
                        self.socks[message["description"]].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
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
                        elif message["description"] == "ODOM":
                            msgType = "nav_msgs/Odometry"
                        else:
                            msgType = message["description"]

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
                            self.socks[message["description"]].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 3)
                            self.socks[message["description"]].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                            self.socks[message["description"]].bind((MCAST_GRP, int(message["port"])))

                            self.threadsLaunched[message["description"]] = threading.Thread(target=recvPubSocket, args=(self.socks[message["description"]], self.ADDR2NAME, message["description"], self.requested[message["description"]], self.metaTopic, MY_NAME))
                            self.threadsLaunched[message["description"]].daemon = True
                            self.threadsLaunched[message["description"]].start()

                            self.topics2PortRx[message["description"]] = int(message["port"])
                            self.portsIUse.append(int(message["port"]))

                            temp = String()
                            temp.data = "end of second inner if"
                            self.debugTopic.publish(temp)

                        if message["description"] in self.threadsLaunched:
                            temp = String()
                            temp.data = "in third inner if "+MCAST_GRP+" "+str(message["port"])
                            self.debugTopic.publish(temp)
                            socks_key = message["description"]+str(message["port"])
                            self.socks[socks_key] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                            self.socks[socks_key].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 3)
                            self.socks[socks_key].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                            self.socks[socks_key].bind((MCAST_GRP, int(message["port"])))

                            self.threadsLaunched[socks_key] = threading.Thread(target=recvPubSocket, args=(self.socks[message["description"]], self.ADDR2NAME, message["description"], self.requested[message["description"]], self.metaTopic, MY_NAME))
                            self.threadsLaunched[socks_key].daemon = True
                            self.threadsLaunched[socks_key].start()
                            temp = String()
                            temp.data = "launched on port "+str(message["port"])
                            self.debugTopic.publish(temp)

                            self.topics2PortRx[socks_key] = message["port"]
                            self.portsIUse.append(int(message["port"]))

                            temp = String()
                            temp.data = "end of third inner if"
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

                    if message["description"] in self.topics2PortTx:
                        newMsg = {}
                        newMsg["TYPE"] = "SEND"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] = message["description"]
                        newMsg["port"] = str(portToUse)
                        newMsg["message_type"] = self.requested[message["description"]]

                        self.socks["meta"].sendto(json.dumps(message), (MCAST_GRP, META_PORT))

                    elif message["description"] in self.waitingFor:
                        portToUse = self.highestPortSeen + 1

                        newMsg = {}
                        newMsg["TYPE"] = "SEND"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] = message["description"]
                        newMsg["port"] = int(portToUse)
                        newMsg["message_type"] = self.requested[message["description"]]

                        def timerCallback(sock = self.socks["meta"], message = newMsg, timersDict = self.timers, port = str(portToUse)):
                            sock.sendto(json.dumps(message), (MCAST_GRP, META_PORT))
                            timersDict[port] = None
                    
                        newMsg = {}
                        newMsg["TYPE"] = "PORT_POLL"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] = message["description"]
                        newMsg["port"] = int(portToUse)
                        newMsg["message_type"] = self.requested[message["description"]]
                        self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

                        self.portsIUse.append(portToUse)
                        self.highestPortSeen += 1

                        self.timers[str(portToUse)] = threading.Timer(POLLTIMERAMOUNT, timerCallback)
                        self.timers[str(portToUse)].start()
                
                elif message["TYPE"] == "PORT_POLL":
                    if int(message["port"]) in self.portsIUse:
                        newMsg = {}
                        newMsg["TYPE"] = "PORT_TAKEN"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] = message["description"]
                        newMsg["port"] = int(message["port"])
                        newMsg["message_type"] = message["message_type"]
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
                        newMsg["port"] = int(portToUse)
                        newMsg["message_type"] = self.requested[message["description"]]

                        def timerCallback(sock = self.socks["meta"], message = newMsg, timersDict = self.timers, port = str(portToUse)):
                            sock.sendto(json.dumps(message), (MCAST_GRP, META_PORT))
                            timersDict[port] = None
                            
                        newMsg = {}
                        newMsg["TYPE"] = "PORT_POLL"
                        newMsg["SENDER"] = MY_NAME
                        newMsg["description"] = message["description"]
                        newMsg["port"] = int(portToUse)
                        newMsg["message_type"] = self.requested[message["description"]]
                        self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

                        self.portsIUse.append(portToUse)
                        self.highestPortSeen += 1

                        self.timers[str(portToUse)] = threading.Timer(POLLTIMERAMOUNT, timerCallback)
                        self.timers[str(portToUse)].start()

                elif message["TYPE"] == "OFFERS_REQ":
                    newMsg = {}
                    newMsg["TYPE"] = "OFFERS_ACK"
                    newMsg["SENDER"] = MY_NAME
                    newMsg["topics"] = TOPICSIHAVE
                    self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

                elif message["TYPE"] == "OFFERS_ACK":
                    for key,data in message["topics"]:
                        topicOfferMsg = GxOffer()
                        topicOfferMsg.description = key
                        topicOfferMsg.type = data
                        self.availableTopic.publish(topicOfferMsg)



            if not self.requestQ.empty():
                temp = String()
                temp.data = "got request"
                self.debugTopic.publish(temp)
                
                newRequest = self.requestQ.get()

                newMsg = {}
                newMsg["TYPE"] = "REQUEST"
                newMsg["SENDER"] = MY_NAME
                newMsg["description"] = newRequest.description
                newMsg["message_type"] = newRequest.type

                self.socks["meta"].sendto(json.dumps(newMsg), (MCAST_GRP, META_PORT))

                def myTimer(timeSec = 10, msg = newMsg, sock = self.socks["meta"]):
                    import time
                    while not rospy.is_shutdown():
                        sock.sendto(json.dumps(msg), (MCAST_GRP, META_PORT))
                        time.sleep(timeSec)

                self.timers["request_"+newRequest.description] = threading.Thread(target=myTimer, args=())
                self.timers["request_"+newRequest.description].daemon = True
                self.timers["request_"+newRequest.description].start()



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