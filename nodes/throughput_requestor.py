#!/usr/bin/env python

import rospy
import StringIO, struct
from std_msgs.msg import String
from gray_transceiver.msg import GxTopicMetaInformation, throughputTest
from gray_transceiver.srv import GxRequest, GxOffer

class requester(object):

    def __init__(self):
        '''
        Constructor for requester class.
        '''
        rospy.init_node("requester")
        self.frequency = rospy.get_param("/throughput_test/frequency_hz")
        self.messageSize = rospy.get_param("/throughput_test/messageSize_bytes")
        self.numberOfBroadcastTopics = rospy.get_param("/throughput_test/broadcastTopics_number")
        self.runTime = rospy.get_param("/throughput_test/runTime_seconds", 300)

        self.data_pub = rospy.Publisher("/throughput_test_data", throughputTest, queue_size = 10)
        
    def run(self):
        '''
        Do the initial requests and then do nothing
        '''

        rospy.wait_for_service('/gray_transceiver/gray_transceiver/offers')
        try:
            offer = rospy.ServiceProxy('/gray_transceiver/gray_transceiver/offers', GxOffer)
            testOffer = GxTopicMetaInformation()
            request = rospy.ServiceProxy('/gray_transceiver/gray_transceiver/requests', GxRequest)
            testRequest = GxTopicMetaInformation()
            for each in range(0, self.numberOfBroadcastTopics):
                testOffer.description = "throughput"+str(each)
                testOffer.type = "gray_transceiver/throughputTest"
                resp3 = offer(testOffer, "/throughput_test_data")
                resp2 = request(testOffer)

        except rospy.ServiceException, e:
            print "test request service call failed: %s"%e
        

        message = throughputTest()
        messageBitString = StringIO.StringIO()
        for each in range(0, self.messageSize):
            message.data.append(each%9)

        #message.data = messageBitString.getvalue()

        messagesToSend = self.frequency * self.runTime

        rate = rospy.Rate(self.frequency)

        for each in range(0, messagesToSend):
            self.data_pub.publish(message)
            rate.sleep()



if __name__ == "__main__":
    Requester = requester()
    Requester.run()