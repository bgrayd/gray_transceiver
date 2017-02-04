#!/usr/bin/env python

import json
import rospy
import socket
import subprocess
import throughput_runner


JSON_VALUES = [True, False]#[True, False]
COMPUTER_VALUES = [5,4,3,2]#[2,3,4,5]
FREQUENCY_VALUES = [10, 15, 20, 25, 30, 50]#[10, 50, 100]#[10,20,30,100,200]
BROADCAST_TOPIC_VALUES = [1,2,3,4,5]
MESSAGE_SIZE_VALUES = [1, 10, 2000, 5000, 10000, 20000,30000]#[10, 20, 30, 40, 100, 1000, 30000]
RUNTIME = 60 #max json is between 21788

basePort = 9000
MY_IP_ADDR = subprocess.check_output(["ifconfig", 'wlan0']).split("inet addr:")[1].split(" ")[0]

class master(object):
    def __init__(self):
        rospy.init_node("throughput_master")

        self.sockets = []
        self.connections = []

        self.setupSockets()

    def setupSockets(self):

        for each in range(0, max(COMPUTER_VALUES)-1):
            print(MY_IP_ADDR)
            print(basePort+each)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.bind((MY_IP_ADDR, basePort + each))
            s.listen(1)
            conn, addr = s.accept()

            self.sockets.append(s)
            self.connections.append(conn)

    def startOthers(self, settings):
        settingsToSend = settings.dump()
        for each in range(0, settings.getComputers()-1):
            self.connections[each].send(settingsToSend)

    def recvFromOthers(self, settings):
        mins = []
        maxs = []
        avgs = []

        for each in range(0, settings.getComputers() - 1):
            data = self.connections[each].recv(1024)
            dataLib = json.loads(data)

            mins.append(dataLib["min"])
            maxs.append(dataLib["max"])
            avgs.append(dataLib["avg"])

        return mins, maxs, avgs

    def stopOthers(self):
        settings = throughput_runner.runSettings()
        settings.setRunTime(-1)
        for each in self.connections:
            each.send(settings.dump())

    def runIteration(self, settings):

        self.startOthers(settings)

        runner = throughput_runner.run()
        runner.loadSettings(settings.dump())
        runner.run()
        runner.addToFile()

        #runner.messageCounts = [settings.getFrequency(), settings.getBroadcastTopicNumber()]

        mins, maxs, avgs = self.recvFromOthers(settings)
        

        mins.append(runner.getMinThroughput())
        maxs.append(runner.getMaxThroughput())
        avgs.append(runner.getAvgThroughput())

        minOfMins = min(mins)
        maxOfMaxs = max(maxs)
        avgOfAvgs = (sum(avgs) * 1.0) / len(avgs)

        print(mins)
        print(maxs)
        print(avgs)

        print("min:"+str(minOfMins))
        print("max:"+str(maxOfMaxs))
        print("avg:"+str(avgOfAvgs))
        print("expected:"+str(settings.getMessageSize()*settings.getBroadcastTopicNumber()*settings.getFrequency()*(settings.getComputers()-1)))

        file2 = open("throughput_test_bandwidths.csv","a")
        file2.write(settings.getAsCsv())
        file2.write(','+str(minOfMins))
        file2.write(","+str(maxOfMaxs))
        file2.write(","+str(avgOfAvgs))
        file2.write(','+str(settings.getMessageSize()*settings.getBroadcastTopicNumber()*settings.getFrequency()*(settings.getComputers()-1)))
        file2.write(",\n")
        file2.close()

    def run(self):

        currentSettings = throughput_runner.runSettings()
        currentSettings.setRunTime(RUNTIME)

        try:
            file1 = open("throughput_test_counts.csv","r")
            file1.close()
        except:
            file2 = open("throughput_test_counts.csv","w")
            file2.write(currentSettings.getCsvHeader()+"\n")
            file2.close()

        try:
            file1 = open("throughput_test_bandwidths.csv","r")
            file1.close()
        except:
            file2 = open("throughput_test_bandwidths.csv","w")
            file2.write(currentSettings.getCsvHeader()+"min,max,avg,expected,\n")
            file2.close()

        currentSettings = throughput_runner.runSettings()
        currentSettings.setRunTime(RUNTIME)
        for jBool in JSON_VALUES:
            currentSettings.setJsonBool(jBool)
            for coms in COMPUTER_VALUES:
                currentSettings.setComputers(coms)
                for freq in FREQUENCY_VALUES:
                    currentSettings.setFrequency(freq)
                    for btn in BROADCAST_TOPIC_VALUES:
                        currentSettings.setBroadcastTopicNumber(btn)
                        for msgSize in MESSAGE_SIZE_VALUES:
                            currentSettings.setMessageSize(msgSize)

                            if jBool and msgSize >= 21788:
                                continue
                            self.runIteration(currentSettings)

        self.stopOthers()





if __name__ == "__main__":
    Master = master()
    Master.run()