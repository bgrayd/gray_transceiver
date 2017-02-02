#!/usr/bin/env python

import json
import rospy
import roslaunch
import StringIO, struct
from std_msgs.msg import String
from gray_transceiver.msg import GxTopicMetaInformation, throughputTest
from gray_transceiver.srv import GxRequest, GxOffer


class runSettings(object):
    def __init__(self):
        self.data = {}

        #default values
        self.data{"computers"} = 0
        self.data{"frequency"} = 10
        self.data{"messageSize"} = 10
        self.data{"broadcastTopics"} = 1
        self.data{"jsonBool"} = False
        self.data{"runTime"} = 300 #5 minutes

    def load(self, newData):
        self.data = json.loads(newData)
    
    def dump(self):
        return json.dumps(self.data)

    def getBagName(self):
        runningName = "throughputTest_"
        
        if self.data{"jsonBool"}:
            runningName += "json_"
        else:
            runningName += "ros_"

        runningName += str(self.data{"computers"})
        runningName += "-coms_"

        runningName += str(self.data{"frequency"})
        runningName += "-hz_"

        runningName += str(self.data{"messageSize"})
        runningName += "-bytes_"

        runningName += str(self.data{"broadcastTopics"})
        runningName += "-copies_"

        runningName += str(self.data{"runTime"})
        runningName += "-sec"

        runningName += ".bag"

        return runningName

    def setFromBagName(self, bagName):
        names = bagName.split("_")

        if names[1] == "json":
            self.data{"jsonBool"} = True
        else:
            self.data{"jsonBool"} = False

        coms = names[2].split("-")
        self.data{"computers"} = coms[0]

        freq = names[3].split("-")
        self.data{"frequency"} = freq[0]

        msgB = names[4].split("-")
        self.data{"messageSize"} = msgB[0]

        broadCast = names[5].split("-")
        self.data{"broadcastTopics"} = broadCast[0]

        runTime = names[6].split("-")
        self.data{"runTime"} = runTime[0]


    def setComputers(self, number):
        self.data{"computers"} = number

    def getComputers(self):
        return self.data{"computers"}

    def setFrequency(self, freq):
        self.data{"frequency"} = freq

    def getFrequency(self):
        return self.data{"frequency"}

    def setMessageSize(self, msgSize):
        self.data{"messageSize"} = msgSize

    def getMessageSize(self):
        return self.data{"messageSize"}

    def setBroadcastTopicNumber(self, number):
        self.data{"broadcastTopics"} = number

    def getBroadCastTopicNumber(self):
        return self.data{"broadcastTopics"}

    def setJsonBool(self, jsonBool):
        self.data{"jsonBool"} = jsonBool

    def getJsonBool(self):
        return self.data{"jsonBool"}

    def setRunTime(self, runTime):
        self.data{"runTime"} = runTime

    def getRunTime(self):
        return self.data{"runTime"}

class run(object):
    def __init__(self):
        self.settings = runSettings()

        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        self.gxNode = None
        self.requestor = None

    def loadSettings(self, setting):
        self.settings.load(setting)

    def setupParameters(self):
        rospy.set_param("throughput_test/frequency_hz", self.settings.getFrequency())
        rospy.set_param("throughput_test/messageSize_bytes", self.settings.getMessageSize())
        rospy.set_param("throughput_test/broadcastTopics_number", self.settings.getBroadCastTopicNumber())
        rospy.set_param("throughput_test/json_bool", self.settings.getJsonBool())
        rospy.set_param("throughput_test/runTime_seconds", self.settings.getRunTime())

    def launchGx(self):
        package = "gray_transceiver"
        executable = "gray_transceiver_main.py"
        nodeNamespace = "/gray_transceiver/"
        nodeName = "gray_transceiver_main""

        arguments = ""

        gxNode = roslaunch.core.Node(package, executable,name=nodeName, namespace=nodeNamespace, args=arguments)
        self.gxNode = self.launch.launch(gxNode)

    def launchRequestor(self):
        package = "gray_transceiver"
        executable = "throughput_requestor.py"
        nodeNamespace = "/gray_transceiver/"
        nodeName = "throughput_requestor"

        arguments = ""

        requestorNode = roslaunch.core.Node(package, executable,name=nodeName, namespace=nodeNamespace, args=arguments)
        self.requestorNode = self.launch.launch(requestorNode)

    def startBag(self):

    #What methods of analysis should be used?

    #end the subscribers, bag, and nodes
    def end(self):
        self.gxNode.stop()
        self.requestorNode.stop()

    def run(self):


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