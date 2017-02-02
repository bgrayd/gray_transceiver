#!/usr/bin/env python

import json
import rospy
import signal
import roslaunch
import subprocess
import StringIO, struct
from std_msgs.msg import String
from gray_transceiver.msg import GxTopicMetaInformation, throughputTest, GxMetaTopic
from gray_transceiver.srv import GxRequest, GxOffer

#this function is based off
#   http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/?answer=223617#post-id-223617
def terminate_process_and_children(p):
    import psutil
    process = psutil.Process(p.pid)
    for sub_process in process.get_children(recursive=True):
        try:
            sub_process.send_signal(signal.SIGINT)
        except:
            pass
    p.wait()  # we wait for children to terminate
    #p.terminate()

class runSettings(object):
    def __init__(self):
        self.data = {}

        #default values
        self.data["computers"] = 0
        self.data["frequency"] = 10
        self.data["messageSize"] = 10
        self.data["broadcastTopics"] = 1
        self.data["jsonBool"] = False
        self.data["runTime"] = 300 #5 minutes

    def load(self, newData):
        self.data = json.loads(newData)
    
    def dump(self):
        return json.dumps(self.data)

    def getBagName(self):
        runningName = "throughputTest_"
        
        if self.data["jsonBool"]:
            runningName += "json_"
        else:
            runningName += "ros_"

        runningName += str(self.data["computers"])
        runningName += "-coms_"

        runningName += str(self.data["frequency"])
        runningName += "-hz_"

        runningName += str(self.data["messageSize"])
        runningName += "-bytes_"

        runningName += str(self.data["broadcastTopics"])
        runningName += "-copies_"

        runningName += str(self.data["runTime"])
        runningName += "-sec"

        runningName += ".bag"

        return runningName

    def setFromBagName(self, bagName):
        names = bagName.split("_")

        if names[1] == "json":
            self.data["jsonBool"] = True
        else:
            self.data["jsonBool"] = False

        coms = names[2].split("-")
        self.data["computers"] = coms[0]

        freq = names[3].split("-")
        self.data["frequency"] = freq[0]

        msgB = names[4].split("-")
        self.data["messageSize"] = msgB[0]

        broadCast = names[5].split("-")
        self.data["broadcastTopics"] = broadCast[0]

        runTime = names[6].split("-")
        self.data["runTime"] = runTime[0]

    def getAsCsv(self):
        line = ""

        line += str(self.data["runTime"])
        line += ","
        
        if self.data["jsonBool"]:
            line += "json,"
        else:
            line += "ros,"

        line += str(self.data["computers"])
        line += ","

        line += str(self.data["frequency"])
        line += ","

        line += str(self.data["broadcastTopics"])
        line += ","

        line += str(self.data["messageSize"])

        return line

    def getCsvHeader(self):
        return "runtime (seconds),serialization,number of computers,publish frequency (Hz),number of broadcast topics,message size (bytes),"

    def setComputers(self, number):
        self.data["computers"] = number

    def getComputers(self):
        return self.data["computers"]

    def setFrequency(self, freq):
        self.data["frequency"] = freq

    def getFrequency(self):
        return self.data["frequency"]

    def setMessageSize(self, msgSize):
        self.data["messageSize"] = msgSize

    def getMessageSize(self):
        return self.data["messageSize"]

    def setBroadcastTopicNumber(self, number):
        self.data["broadcastTopics"] = number

    def getBroadcastTopicNumber(self):
        return self.data["broadcastTopics"]

    def setJsonBool(self, jsonBool):
        self.data["jsonBool"] = jsonBool

    def getJsonBool(self):
        return self.data["jsonBool"]

    def setRunTime(self, runTime):
        self.data["runTime"] = runTime

    def getRunTime(self):
        return self.data["runTime"]

class run(object):
    def __init__(self):
        self.settings = runSettings()

        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        self.waitStart = self.settings.getRunTime()/10.0
        self.calcFreq = 10

        self.gxNode = None
        self.requestor = None

        self.bagger = None

        self.messageCount = 0
        self.messageCounts = []

        self.subscribers = []
        self.subscribers.append(rospy.Subscriber("gray_transceiver/metatopic", GxMetaTopic, self.metaTopic_callback))


    def metaTopic_callback(self, data):
        self.subscribers.append(rospy.Subscriber(data.name, throughputTest, self.throughputCount_callback))

    def throughputCount_callback(self, data):
        self.messageCount += 1

    def loadSettings(self, setting):
        self.settings.load(setting)

    def setupParameters(self):
        rospy.set_param("throughput_test/frequency_hz", self.settings.getFrequency())
        rospy.set_param("throughput_test/messageSize_bytes", self.settings.getMessageSize())
        rospy.set_param("throughput_test/broadcastTopics_number", self.settings.getBroadcastTopicNumber())
        rospy.set_param("throughput_test/json_bool", self.settings.getJsonBool())
        rospy.set_param("throughput_test/runTime_seconds", self.settings.getRunTime())

    def launchGx(self):
        package = "gray_transceiver"
        executable = "gray_transceiver_main.py"
        nodeNamespace = "/gray_transceiver/"
        nodeName = "gray_transceiver_main"

        arguments = ""

        gxNode = roslaunch.core.Node(package, executable,name=nodeName, namespace=nodeNamespace, args=arguments)
        self.gxNode = self.launch.launch(gxNode)

    def launchRequestor(self):
        package = "gray_transceiver"
        executable = "throughput_requestor.py"
        nodeNamespace = "/"
        nodeName = "throughput_requestor"

        arguments = ""

        requestorNode = roslaunch.core.Node(package, executable,name=nodeName, namespace=nodeNamespace, args=arguments)
        self.requestorNode = self.launch.launch(requestorNode)

    def startBag(self):
        #command = ['rosbag', 'record', '-e', '"/foreign_(.*)/throughput(.*)"', '-o', '~/Gx_bags/'+str(self.settings.getBagName())]
        #command = 'rosbag record -e "/foreign_(.*)/throughput(.*)" -o ~/Gx_bags/'+str(self.settings.getBagName())
        command = 'rosbag record -e "/foreign_(.*)/throughput(.*)" -o' + str(self.settings.getBagName())
        self.bagger = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,shell=True, cwd="/home/billy/Gx_bags")

    def stopSubscribers(self):
        for each in self.subscribers:
            each.unregister()
        self.subscribers = []

    #end the subscribers, bag, and nodes
    def end(self):
        self.gxNode.stop()
        self.requestorNode.stop()
        self.stopSubscribers()

        #self.bagger.terminate()
        terminate_process_and_children(self.bagger)

    def addToFile(self):
        line = self.settings.getAsCsv()
        for each in self.messageCounts:
            line += ','
            line += str(each)

        line += '\n'

        file1 = open("throughput_test_counts.csv", 'a')
        file1.write(line)
        file1.close()

    def run(self):
        self.waitStart = self.settings.getRunTime()/10.0 #recalculate once the settings are set

        self.setupParameters()
        self.launchGx()
        self.startBag()
        self.launchRequestor()

        rospy.sleep(self.waitStart)

        countsToRun = self.settings.getRunTime() * 0.8 * self.calcFreq
        count = 0

        rate = rospy.Rate(self.calcFreq)
        self.messageCount = 0 #clear all that happened while waiting

        while count < countsToRun:
            rate.sleep()
            
            self.messageCounts.append(self.messageCount)
            self.messageCount = 0

            count += 1

        rospy.sleep(self.waitStart)

        self.end()

    def getMinThroughput(self):
        return min(self.messageCounts) * self.settings.getMessageSize() * self.calcFreq

    def getMaxThroughput(self):
        return max(self.messageCounts) * self.settings.getMessageSize() * self.calcFreq

    def getAvgThroughput(self):
        return (sum(self.messageCounts) * self.settings.getMessageSize() * self.calcFreq) / len(self.messageCounts)


class client(object):

    def __init__(self):
        '''
        Constructor for client class.
        '''
        rospy.init_node("client")

    def startFile(self):
        #see if it exists, if it doesn't, make one
        try:
            file1 = open("throughput_test_counts.csv","r")
            file1.close()
        except:
            file2 = open("throughput_test_counts.csv","w")
            settings = runSettings()
            file2.write(settings.getCsvHeader()+"\n")
            file2.close()
        
    def run(self):
        '''
        Do the initial requests and then do nothing
        '''


if __name__ == "__main__":
    Client = client()
    Client.run()