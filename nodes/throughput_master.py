#!/usr/bin/env python

import json
import rospy
import throughput_runner

JSON_VALUES = [True, False]#[True, False]
COMPUTER_VALUES = [2]#[2,3,4,5]
FREQUENCY_VALUES = [10,20,30]#[10, 50, 100]#[10,20,30,100,200]
BROADCAST_TOPIC_VALUES = [5]#[1,2,3,4,5]
MESSAGE_SIZE_VALUES = [7500]#[10, 20, 30, 40, 100, 1000, 30000]
RUNTIME = 20 #max json is between 21788

class master(object):
    def __init__(self):
        rospy.init_node("throughput_master")

    def runIteration(self, settings):

        runner = throughput_runner.run()
        runner.loadSettings(settings.dump())
        runner.run()
        runner.addToFile()

        print("min:"+str(runner.getMinThroughput()))
        print("max:"+str(runner.getMaxThroughput()))
        print("avg:"+str(runner.getAvgThroughput()))
        print("expected:"+str(settings.getMessageSize()*settings.getBroadcastTopicNumber()*settings.getFrequency()*(settings.getComputers()-1)))

        file2 = open("throughput_test_bandwidths.csv","a")
        file2.write(settings.getAsCsv())
        file2.write(','+str(runner.getMinThroughput()))
        file2.write(","+str(runner.getMaxThroughput()))
        file2.write(","+str(runner.getAvgThroughput()))
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





if __name__ == "__main__":
    Master = master()
    Master.run()