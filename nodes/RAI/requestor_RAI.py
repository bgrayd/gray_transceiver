#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gray_transceiver.msg import GxTopicMetaInformation
from gray_transceiver.srv import GxRequest, GxOffer
from sensor_msgs.msg import CompressedImage, CameraInfo

class requester(object):

    def __init__(self):
        '''
        Constructor for requester class.
        '''
        rospy.init_node("requester")
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.image_sub)
        rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camInfo_sub)

        self.newImage = False
        self.newInfo = False

        self.image_pub = rospy.Publisher("/kinectImage/compressed", CompressedImage, queue_size = 10)
        self.camInfo_pub = rospy.Publisher("/kinectImage/camera_info", CameraInfo, queue_size = 10)

    def image_sub(self, data):
        self.message = data
        self.newImage = True

    def camInfo_sub(self, data):
        self.camInfo = data
        self.newInfo = True
        
    def run(self):
        '''
        Do the initial requests and then do nothing
        '''

        rospy.wait_for_service('gray_transceiver/offers')
        try:
            offer = rospy.ServiceProxy('gray_transceiver/offers', GxOffer)
            odomOffer = GxTopicMetaInformation()
            odomOffer.description = "kinectCamera/compressed"
            odomOffer.type = "sensor_msgs/CompressedImage"
            resp3 = offer(odomOffer, "/kinectImage/compressed")
        except rospy.ServiceException, e:
            print "Odom offer service call failed: %s"%e

        try:
            offer = rospy.ServiceProxy('gray_transceiver/offers', GxOffer)
            odomOffer = GxTopicMetaInformation()
            odomOffer.description = "kinectCamera/camera_info"
            odomOffer.type = "sensor_msgs/CameraInfo"
            #resp3 = offer(odomOffer, "/kinectImage/camera_info")
        except rospy.ServiceException, e:
            print "Odom offer service call failed: %s"%e

        rospy.wait_for_service('gray_transceiver/requests')
        try:
            request = rospy.ServiceProxy('gray_transceiver/requests', GxRequest)
            odomRequest = GxTopicMetaInformation()
            odomRequest.description = "move_cmd"
            odomRequest.type = "gray_transceiver/twistCommand"
            resp1 = request(odomRequest)
        except rospy.ServiceException, e:
            print "Odom request service call failed: %s"%e

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.newInfo:
                self.camInfo_pub.publish(self.camInfo)
            if self.newImage:
                self.image_pub.publish(self.message)
            rate.sleep()

if __name__ == "__main__":
    Requester = requester()
    Requester.run()
