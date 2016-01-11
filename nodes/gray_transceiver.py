#!/usr/bin/env python

import rospy
import socket
import threading
from Queue import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

MCAST_GRP = '224.1.1.1' #change this to use a parameter
#possibly rename this base port
META_PORT = 1025           #possibly change this to a parameter
MY_NAME = rospy.get_param("gray_transceiver/my_name", "robot")#"NOTCHANGED")
TOPICSIHAVE = rospy.get_param("gray_transceiver/topics_i_have",{"LIDAR":"/scan", "ODOM":"/odom"})

#######################################################################
#Please remove this part after the proof of concept testing.........
PoC_ODOM_Port = 2000   #this needs to be removed after PoC testing
PoC_LIDAR_Port = 3000  #this needs to be removed after PoC testing
#More needs to be removed far down
#######################################################################



#####################################
#split has the ones being split be in it's own element (python 2.7)
####################################

def recvPubSocket(sock, addr2Name, topicName, messageType, metaTopic, maxsize = 1024):
    publishers = {}
    while True:
        try:
            data, addr = sock.recvfrom(maxsize)
        except socket.error, e:
            print 'Exception'
            continue
        senderDomain = addr2Name[str(addr)]
        if senderDomain in publishers:
            publishers[senderDomain].publish(data)
        else:
            publishers[senderDomain] = rospy.Publisher(str(senderDomain)+'/'+str(topicName), messageType)
            metaTopic.publish(str(senderDomain)+'/'+str(topicName))
            publishers[senderDomain].publish(data)

def recvQueSocket(sock, queue, maxsize = 1024):
    while True:
        try:
            data, addr = sock.recvfrom(maxsize)
        except socket.error, e:
            print 'Exception'
        queue.put(str(data)+"~"+str(addr))



class gray_transceiver(object):

    def __init__(self):
        print("here 1")
        rospy.init_node("gray_transceiver")
        self.requestQ = Queue(10)
        self.metaSockQ = Queue(20)
        self.socks = {}
        self.threadsLaunched = {}
        self.topics2PortTx = {}
        self.topics2PortRx = {}
        self.ports2topic = {}
        self.nextPort = 2
        self.names = []
        self.requested = {}

        #this might be removed after PoC, currently unsure
        self.waitingFor = []

        #translate the IP address to the name of the sender
        self.ADDR2NAME = {}

        self.socks["meta"] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socks["meta"].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)
        self.socks["meta"].bind((MCAST_GRP, META_PORT))
        self.host = socket.gethostbyname(socket.gethostname())
        self.socks["meta"].setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))
        self.socks["meta"].setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(self.host))

        self.threadsLaunched["meta"] = threading.Thread(target=recvQueSocket, args=(self.socks["meta"], self.metaSockQ))
        self.threadsLaunched["meta"].daemon = True
        self.threadsLaunched["meta"].start()

        self.metaTopic = rospy.Publisher(METATOPICNAME, String)
        rospy.Subscriber("gray_transceiver/requests", String, self.requests_callback)


        self.socks["meta"].sendto('NEW~'+MY_NAME, (MCAST_GRP, META_PORT))
        print("here 2")

    #look into seeing if the message can just be pu directly in the queue
    def requests_callback(self, data):
        '''
        This function gets called everytime a message is published over the /gray_transceiver/requests topic.
        '''
        self.requestQ.put(data)
        
    def run(self):
        accepted = False
        attempt = -1
        for i in range(100):
            if self.metaSockQ.empty():
                noResponse = True
                rospy.sleep(0.001)
            else:
                noResponse = False
                break
        
        if noResponse:
            accepted = True

        while not accepted:
            message = self.metaSockQ.get()
            messageParts = message.split("~")
            if messageParts[0] == "ACCEPT":
                if messageParts[1] == MY_NAME:
                    accepted = True

                elif messageParts[1] == MY_NAME+str(attempt):
                    accepted = True
                    MY_NAME = MY_NAME+str(attempt)
            elif messageParts[0] == "DENY":
                if messageParts[1] == MY_NAME:
                    attempt += 1
                    self.socks["meta"].sendto('NEW~'+MY_NAME+str(attempt), (MCAST_GRP, META_PORT))

                elif messageParts[1] == MY_NAME+str(attempt):
                    attempt += 1
                    self.socks["meta"].sendto('NEW~'+MY_NAME+str(attempt), (MCAST_GRP, META_PORT))
            else:
                self.metaSockQ.put(message)

        self.names.append(MY_NAME)

        while not rospy.is_shutdown():
            hadMessage = False
            if not self.metaSockQ.empty():
                message = self.metaSockQ.get()
                messages = message.split("~")

                while "~" in messages:
                    messages.remove("~")

                if messages[0] == "NEW":
                    if messages[1] in self.names:
                        self.socks["meta"].sendto('DENY~'+messages[1], (MCAST_GRP, META_PORT))
                    else:
                        self.socks["meta"].sendto('ACCEPT~'+messages[1], (MCAST_GRP, META_PORT))
                        self.names.append(messages[1])
                        self.ADDR2NAME[messages[2]] = messages[1]
                        self.socks["meta"].sendto("IAM~"+MY_NAME, (MCAST_GRP, META_PORT))

                elif messages[0] == "IAM":
                    if messages[1] not in self.names:
                        self.names.append(messages[1])

                elif messages[0] == "REQUEST":
                    #this is where they will ask each other for sending information, for now options are LIDAR and ODOM
                    #   and should not be garunteed to have it
                    if messages[1] in self.topics2PortTx:
                        self.socks["meta"].sendto("TXING~"+messages[1]+"~"+str(self.topics2PortTx[messages[1]]), (MCAST_GRP, META_PORT))

                    elif messages[1] == "LIDAR":
                        self.socks["meta"].sendto("IHAVE~"+messages[1],(MCAST_GRP, META_PORT))

                    elif messages[1] == "ODOM":
                        self.socks["meta"].sendto("IHAVE~"+messages[1], (MCAST_GRP, META_PORT))

                elif messages[0] == "SEND":
                    #this is where it starts call backs for sending messages
                    if messages[1] in self.topics2PortTx:
                        #do nothing cause you already are transmitting it
                        pass
                    elif (messages[1] == "LIDAR" or messages[1] == "ODOM"):
                        if messages[1] not in self.socks:
                            self.socks[messages[1]] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

                        self.host = socket.gethostbyname(socket.gethostname())
                        self.socks[messages[1]].setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))
                        self.socks[messages[1]].setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(self.host))

                        def dynamicCallback(data):
                            self.socks[messages[1]].sendto(data, (MCAST_GRP, messages[2]))

                        self.socks["meta"].sendto("TXING~"+messages[1]+"~"+messages[2], (MCAST_GRP, META_PORT))
                        if messages[1] == "LIDAR":
                            msgType = "sensor_msgs/LaserScan"
                        else:
                            msgType = "nav_msgs/Odometry"
                        # myType = getattr(__import__("sensor_msgs.msg", fromlist=["LaserScan"], level=1), "LaserScan")
                        msgType.strip("/")
                        msgTypes = msgType.split("/")
                        while "/" in msgTypes:
                            msgTypes.remove("/")
                        myType = getattr(__import__(str(msgTypes[0])+".msg", fromlist=[msgTypes[1]], level=1), msgTypes[1])
                        rospy.Subscriber(TOPICSIHAVE[messages[1]], myType, self.requests_callback)

                elif messages[0] == "TXING":
                    #if messages[1] is something you want, launch a thread and listen to it and publish
                    if messages[1] in self.requests:
                        if messages[1] not in self.socks:
                            self.socks[messages[1]] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                        if messages[1] not in self.threadsLaunched:
                            self.socks[messages[1]].setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)
                            self.socks[messages[1]].bind((MCAST_GRP, META_PORT))

                            self.threadsLaunched[messages[1]] = threading.Thread(target=recvPubSocket, args=(self.socks[messages[1]], self.ADDR2NAME, messages[1], self.requested[messages[1]], self.metaTopic))
                            self.threadsLaunched[messages[1]].daemon = True
                            self.threadsLaunched[messages[1]].start()

                elif messages[0] == "IHAVE":
                    
                    #check against a list of messages you want but haven't heard back about yet
                    #if someone has something you want, set a timer, for a small amount of time,
                    #   after the timer goes off, if no one has said they are already txing it, 
                    #   send out a poll for the next port to use
                    #may also want the Txing to keep a dictionary of each that it has seen, with the port as the key

                    ###############################################################################
                    #please remove this part after PoC testing
                    #are you waiting for the thing someone offered?
                    if messages[1] in self.waitingFor:
                        portToUse = META_PORT+1
                        if messages[1] == "LIDAR":
                            portToUse = PoC_LIDAR_Port
                        if messages[1] == "ODOM":
                            portToUse = PoC_ODOM_Port

                        self.socks["meta"].sendto("SEND~"+messages[1]+"~"+str(portToUse), (MCAST_GRP, META_PORT))


                    ###############################################################################

                #max port number is 65535



            if not self.requestQ.empty():
                newRequest = self.requestQ.get()
                newRequest = newRequest.split("~")
                #later on, have more than just type, such as when received locally publish on 1 topic total or 1 per source
                #   also possibly metaish data about it
                #   currently name~type
                while "~" in newRequest:
                    newRequest.remove("~")

                self.socks["meta"].sendto("REQUEST~"+newRequest[0],(MCAST_GRP, META_PORT))

                msgTypes = newRequest[1].split("/")
                while "/" in msgTypes:
                    msgTypes.remove("/")
                myType = getattr(__import__(str(msgTypes[0])+".msg", fromlist=[msgTypes[1]], level=1), msgTypes[1]) #put in try, can throw error if it doesn't have the attribute, or use hasattr(object, name)

                self.requested[newRequest[0]] = myType

                self.waitingFor.append(myType) #might be able to remove after PoC testing

            rospy.sleep(0.1)                    # sleep briefly so ROS doesn't die



#consider changing meta topic to have a latch
#   or use rospy.timer and have the callback publish
if __name__ == "__main__":
    grayTransceiver = gray_transceiver()
    grayTransceiver.run()