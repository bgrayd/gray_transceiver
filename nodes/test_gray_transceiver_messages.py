#!/usr/bin/env python2
PKG='gray_transceiver'
NAME='test_gray_transceiver_messages'

import unittest
import rostest
from geometry_msgs.msg import Point
from gray_transceiver_message import *



class TestMessages(unittest.TestCase):
    def assertListAlmostEqual(self, list1, list2, tol = 7):
        self.assertEqual(len(list1), len(list2))
        for a, b in zip(list1, list2):
             self.assertAlmostEqual(a, b, tol)

    def test_Request(self):
        message1 = GxRequestMsg(sender = "test", description = "test2", rosMsgType = "test3")
        jsoned1 = message1.toJSON()
        message2 = GxRequestMsg()
        message2.fromJSON(jsoned1)

        self.assertTrue(message1.isRequest())
        self.assertFalse(message1.isSend())
        self.assertFalse(message1.isIHave())
        self.assertFalse(message1.isTxing())
        self.assertFalse(message1.isOffersReq())
        self.assertFalse(message1.isOffersAck())
        self.assertFalse(message1.isData())
        self.assertEqual(message1.getSender(), "test")
        self.assertEqual(message1.getDescription(), "test2")
        self.assertEqual(message1.getRosMsgType(), "test3")

        self.assertTrue(message2.isRequest())
        self.assertFalse(message2.isSend())
        self.assertFalse(message2.isIHave())
        self.assertFalse(message2.isTxing())
        self.assertFalse(message2.isOffersReq())
        self.assertFalse(message2.isOffersAck())
        self.assertFalse(message2.isData())
        self.assertEqual(message2.getSender(), "test")
        self.assertEqual(message2.getDescription(), "test2")
        self.assertEqual(message2.getRosMsgType(), "test3")

    def test_Send(self):
        message1 = GxSendMsg(sender = "test", description = "test2", rosMsgType = "test3")
        jsoned1 = message1.toJSON()
        message2 = GxSendMsg()
        message2.fromJSON(jsoned1)

        self.assertFalse(message1.isRequest())
        self.assertTrue(message1.isSend())
        self.assertFalse(message1.isIHave())
        self.assertFalse(message1.isTxing())
        self.assertFalse(message1.isOffersReq())
        self.assertFalse(message1.isOffersAck())
        self.assertFalse(message1.isData())
        self.assertEqual(message1.getSender(), "test")
        self.assertEqual(message1.getDescription(), "test2")
        self.assertEqual(message1.getRosMsgType(), "test3")

        self.assertFalse(message2.isRequest())
        self.assertTrue(message2.isSend())
        self.assertFalse(message2.isIHave())
        self.assertFalse(message2.isTxing())
        self.assertFalse(message2.isOffersReq())
        self.assertFalse(message2.isOffersAck())
        self.assertFalse(message2.isData())
        self.assertEqual(message2.getSender(), "test")
        self.assertEqual(message2.getDescription(), "test2")
        self.assertEqual(message2.getRosMsgType(), "test3")

    def test_IHave(self):
        message1 = GxIHaveMsg(sender = "test", description = "test2", rosMsgType = "test3")
        jsoned1 = message1.toJSON()
        message2 = GxIHaveMsg()
        message2.fromJSON(jsoned1)

        self.assertFalse(message1.isRequest())
        self.assertFalse(message1.isSend())
        self.assertTrue(message1.isIHave())
        self.assertFalse(message1.isTxing())
        self.assertFalse(message1.isOffersReq())
        self.assertFalse(message1.isOffersAck())
        self.assertFalse(message1.isData())
        self.assertEqual(message1.getSender(), "test")
        self.assertEqual(message1.getDescription(), "test2")
        self.assertEqual(message1.getRosMsgType(), "test3")

        self.assertFalse(message2.isRequest())
        self.assertFalse(message2.isSend())
        self.assertTrue(message2.isIHave())
        self.assertFalse(message2.isTxing())
        self.assertFalse(message2.isOffersReq())
        self.assertFalse(message2.isOffersAck())
        self.assertFalse(message2.isData())
        self.assertEqual(message2.getSender(), "test")
        self.assertEqual(message2.getDescription(), "test2")
        self.assertEqual(message2.getRosMsgType(), "test3")

    def test_Txing(self):
        message1 = GxTxingMsg(sender = "test", description = "test2", rosMsgType = "test3")
        jsoned1 = message1.toJSON()
        message2 = GxTxingMsg()
        message2.fromJSON(jsoned1)

        self.assertFalse(message1.isRequest())
        self.assertFalse(message1.isSend())
        self.assertFalse(message1.isIHave())
        self.assertTrue(message1.isTxing())
        self.assertFalse(message1.isOffersReq())
        self.assertFalse(message1.isOffersAck())
        self.assertFalse(message1.isData())
        self.assertEqual(message1.getSender(), "test")
        self.assertEqual(message1.getDescription(), "test2")
        self.assertEqual(message1.getRosMsgType(), "test3")

        self.assertFalse(message2.isRequest())
        self.assertFalse(message2.isSend())
        self.assertFalse(message2.isIHave())
        self.assertTrue(message2.isTxing())
        self.assertFalse(message2.isOffersReq())
        self.assertFalse(message2.isOffersAck())
        self.assertFalse(message2.isData())
        self.assertEqual(message2.getSender(), "test")
        self.assertEqual(message2.getDescription(), "test2")
        self.assertEqual(message2.getRosMsgType(), "test3")

    def test_Data(self):
        pointMsg = Point()
        message1 = GxDataMsg(sender = "test", description = "test2", rosMsgType = "geometry_msgs/Point", data=pointMsg)
        jsoned1 = message1.toJSON()
        bitStringed1 = message1.toBitString()
        
        message2 = GxDataMsg()
        message2.fromJSON(jsoned1)

        message3 = GxDataMsg()
        message3.fromBitString(bitStringed1)

        self.assertFalse(message1.isRequest())
        self.assertFalse(message1.isSend())
        self.assertFalse(message1.isIHave())
        self.assertFalse(message1.isTxing())
        self.assertFalse(message1.isOffersReq())
        self.assertFalse(message1.isOffersAck())
        self.assertTrue(message1.isData())
        self.assertEqual(message1.getSender(), "test")
        self.assertEqual(message1.getDescription(), "test2")
        self.assertEqual(message1.getRosMsgType(), "geometry_msgs/Point")

        self.assertFalse(message2.isRequest())
        self.assertFalse(message2.isSend())
        self.assertFalse(message2.isIHave())
        self.assertFalse(message2.isTxing())
        self.assertFalse(message2.isOffersReq())
        self.assertFalse(message2.isOffersAck())
        self.assertTrue(message2.isData())
        self.assertEqual(message2.getSender(), "test")
        self.assertEqual(message2.getDescription(), "test2")
        self.assertEqual(message2.getRosMsgType(), "geometry_msgs/Point")

        self.assertFalse(message3.isRequest())
        self.assertFalse(message3.isSend())
        self.assertFalse(message3.isIHave())
        self.assertFalse(message3.isTxing())
        self.assertFalse(message3.isOffersReq())
        self.assertFalse(message3.isOffersAck())
        self.assertTrue(message3.isData())
        self.assertEqual(message3.getSender(), "test")
        self.assertEqual(message3.getDescription(), "test2")
        self.assertEqual(message3.getRosMsgType(), "geometry_msgs/Point")

        self.assertEqual(message1.getDataAsDict(), message2.getDataAsDict())
        self.assertEqual(message1.getDataAsDict(), message3.getDataAsDict())



if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestMessages)