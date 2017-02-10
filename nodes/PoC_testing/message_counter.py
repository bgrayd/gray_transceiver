#!/usr/bin/env python

import json
import rospy
import signal
import socket
import roslaunch
import subprocess
import roslib.message
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

class run(object):
    def __init__(self):
        rospy.init_node("message_counter")

        self.calcFreq = 1

        self.bagger = None

        self.currentIndex = 0
        self.messageCounts = []
        self.messageCountsOld = []
        self.columnNames = ["seconds"]

        self.subscribers = []
        self.subscribers.append(rospy.Subscriber("gray_transceiver/metatopic", GxMetaTopic, self.metaTopic_callback))
        rospy.set_param("throughput_test/json_bool", False)



    def metaTopic_callback(self, data):
        pos = self.currentIndex
        self.currentIndex += 1
        self.messageCounts.append(0)
        def count_callback(data, messageCounts = self.messageCounts, index = pos):
            messageCounts[pos] += 1

        myType = roslib.message.get_message_class(data.type)
        self.subscribers.append(rospy.Subscriber(data.name, myType, count_callback))
        self.columnNames.append(data.name)

    def startBag(self):
        command = 'rosbag record -e "/foreign_(.*)/"'
        self.bagger = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,shell=True, cwd="/home")

    def stopSubscribers(self):
        for each in self.subscribers:
            each.unregister()
        self.subscribers = []

    #end the subscribers, bag, and nodes
    def end(self):
        self.stopSubscribers()
        terminate_process_and_children(self.bagger)
        self.makeFile()

    def makeFile(self):
        file1 = open("simTest_counts.csv", "w")
        for each in self.columnNames:
            file1.write(str(each))
            file1.write(",")

        file1.write("\n")
        for i in self.messageCountsOld:
            for j in i:
                file1.write(str(j))
                file1.write(",")
            file1.write("\n")

        file1.close()


    def run(self):

        countsToWait = 25
        count = 0

        rate = rospy.Rate(self.calcFreq)
        rate.sleep()

        while count < countsToWait:
            rate.sleep()
            for each in range(0,self.currentIndex):
                self.messageCounts[each] = 0
            count += 1

        while not rospy.is_shutdown():
            rate.sleep()
            newCounts = []
            for each in range(0,self.currentIndex):
                newCounts.append(self.messageCounts[each])
                self.messageCounts[each] = 0
            self.messageCountsOld.append(newCounts)


        self.end()

if __name__ == "__main__":
    Run = run()
    Run.run()