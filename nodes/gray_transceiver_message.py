#!/usr/bin/env python

import json
import StringIO, struct
import roslib
from gray_transceiver.msg import GxTopicMetaInformation
from rospy_message_converter import message_converter, json_message_converter

class GxBaseMsg(object):
    def __init__(self, sender = None, msgType = None):
        self.sender = sender
        self.msgType = msgType

    def setSender(self, sender):
        self.sender = sender

    def getSender(self):
        return self.sender

    def getType(self):
        return self.msgType

    def fromDict(self, srcDict):
        self.sender = srcDict["SENDER"]
        self.msgType = srcDict["TYPE"]

    def fromJSON(self, srcJSON):
        self.fromDict(json.loads(srcJSON))

    def toDict(self):
        return {"TYPE":self.msgType, "SENDER":self.sender}

    def toJSON(self):
        return json.dumps(self.toDict())

    def isRequest(self):
        return False

    def isSend(self):
        return False

    def isIHave(self):
        return False

    def isTxing(self):
        return False

    def isOffersReq(self):
        return False

    def isOffersAck(self):
        return False

    def isData(self):
        return False

class GxTopicMetaInfoMsg(GxBaseMsg):
    def __init__(self, sender=None, msgType=None, description=None, rosMsgType=None):
        super(GxTopicMetaInfoMsg, self).__init__(sender, msgType)
        self.description = description
        self.rosMsgType = rosMsgType

    def toDict(self):
        toBeReturned = super(GxTopicMetaInfoMsg, self).toDict()
        toBeReturned["description"] = self.description
        toBeReturned["message_type"] = self.rosMsgType
        return toBeReturned

    def fromDict(self, srcDict):
        super(GxTopicMetaInfoMsg, self).fromDict(srcDict)
        self.description = srcDict["description"]
        self.rosMsgType = srcDict["message_type"]

    def setDescription(self, description):
        self.description = description

    def getDescription(self):
        return self.description

    def setRosMsgType(self, rosMsgType):
        self.rosMsgType = rosMsgType

    def getRosMsgType(self):
        return self.rosMsgType

    def getTopicMetaInformation(self):
        data = GxTopicMetaInformation()
        data.description = self.description
        data.type = self.rosMsgType
        return data

class GxRequestMsg(GxTopicMetaInfoMsg):
    def __init__(self, sender=None, description=None, rosMsgType=None):
        super(GxRequestMsg, self).__init__(sender, "REQUEST", description, rosMsgType)

    def isRequest(self):
        return True

class GxSendMsg(GxTopicMetaInfoMsg):
    def __init__(self, sender=None, description=None, rosMsgType=None):
        super(GxSendMsg, self).__init__(sender, "SEND", description, rosMsgType)
    def isSend(self):
        return True

class GxIHaveMsg(GxTopicMetaInfoMsg):
    def __init__(self, sender=None, description=None, rosMsgType=None):
        super(GxIHaveMsg, self).__init__(sender, "IHAVE", description, rosMsgType)
    def isIHave(self):
        return True

class GxTxingMsg(GxTopicMetaInfoMsg):
    def __init__(self, sender=None, description=None, rosMsgType=None):
        super(GxTxingMsg, self).__init__(sender, "TXING", description, rosMsgType)
    def isTxing(self):
        return True

class GxDataMsg(GxTopicMetaInfoMsg):
    def __init__(self, sender=None, description=None, rosMsgType=None, data=None):
        super(GxDataMsg, self).__init__(sender, "DATA", description, rosMsgType)
        self.data = data

    def toDict(self):
        toBeReturned = super(GxDataMsg, self).toDict()
        toBeReturned["data"] = self.getDataAsDict()
        return toBeReturned

    def fromDict(self, srcDict):
        super(GxDataMsg, self).fromDict(srcDict)
        self.setDataFromDict(srcDict["data"])

    def fromSocket(self, source):
        #self.fromJSON(source)
        self.fromBitString(source)

    def toSocket(self):
        #return self.toJSON()
        return self.toBitString()

    def fromBitString(self, data):
        (positionSender, positionType, positionDescription, positionRosMsgType, positionData) = struct.unpack('<IIIII',data[:20])
        serializedSender = struct.unpack('<%ss'%(positionType-positionSender),data[positionSender:positionType])[0]
        serializedType = struct.unpack('<%ss'%(positionDescription-positionType),data[positionType:positionDescription])[0]
        serializedDescription = struct.unpack('<%ss'%(positionRosMsgType-positionDescription), data[positionDescription:positionRosMsgType])[0]
        serializedRosMsgType = struct.unpack('<%ss'%(positionData - positionRosMsgType), data[positionRosMsgType:positionData])[0]
        serializedData = data[positionData:]

        self.setSender(serializedSender)
        self.setDescription(serializedDescription)
        self.setRosMsgType(serializedRosMsgType)
        dataType = roslib.message.get_message_class(self.getRosMsgType())
        self.data = dataType()
        self.data.deserialize(serializedData)

    def toBitString(self):
        serializedSender = self.getSender().encode('utf-8')
        serializedType = self.getType().encode('utf-8')
        serializedDescription = self.getDescription().encode('utf-8')
        serializedRosMsgType = self.getRosMsgType().encode('utf-8')

        serializedData = StringIO.StringIO()
        self.data.serialize(serializedData)

        lengthSender = len(serializedSender)
        lengthType = len(serializedType)
        lengthDescription = len(serializedDescription)
        lengthRosMsgType = len(serializedRosMsgType)
        lengthData = len(serializedData.getvalue())

        positionSender = 20 #(size of each position) * number of positions = 4 * 5
        positionType = positionSender + lengthSender
        positionDescription = positionType + lengthType
        positionRosMsgType = positionDescription + lengthDescription
        positionData = positionRosMsgType + lengthRosMsgType

        runningBitString = StringIO.StringIO()

        runningBitString.write(struct.pack('<I',positionSender))
        runningBitString.write(struct.pack('<I',positionType))
        runningBitString.write(struct.pack('<I',positionDescription))
        runningBitString.write(struct.pack('<I',positionRosMsgType))
        runningBitString.write(struct.pack('<I',positionData))
        runningBitString.write(struct.pack('<%ss'%lengthSender, serializedSender))
        runningBitString.write(struct.pack('<%ss'%lengthType, serializedType))
        runningBitString.write(struct.pack('<%ss'%lengthDescription, serializedDescription))
        runningBitString.write(struct.pack('<%ss'%lengthRosMsgType, serializedRosMsgType))
        runningBitString.write(struct.pack('<%ss'%lengthData, serializedData.getvalue()))

        return runningBitString.getvalue()

    def setDataFromDict(self, data):
        try:
            self.data = message_converter.convert_dictionary_to_ros_message(self.rosMsgType, data)
        except Exception as ex:
            self.data = message_converter.convert_dictionary_to_ros_message(self.rosMsgType, json.loads(data))

    def setDataFromRosMsg(self, rosMsg):
        self.data = rosMsg

    def getDataAsDict(self):
        return message_converter.convert_ros_message_to_dictionary(self.data)

    def getDataAsRosMsg(self):
        return self.data

    def isData(self):
        return True

class GxOffersReqMsg(GxBaseMsg):
    def __init__(self, sender=None):
        super(GxOffersRegMsg, self).__init__(sender, "OFFERS_REQ")

    def isOffersReq(self):
        return True

class GxOffersAckMsg(GxBaseMsg):
    def __init__(self, sender=None, topics=None):
        super(GxOffersAckMsg, self).__init__(sender, "OFFERS_ACK")
        self.topics = topics

    def setTopics(self, newTopics):
        self.topics = newTopics

    def getTopics(self):
        return self.topics

    def toDict(self):
        toBeReturned = super(GxOffersAckMsg, self).toDict()
        toBeReturned["topics"] = self.topics
        return toBeReturned

    def fromDict(self, srcDict):
        super(GxOffersAckMsg, self).fromDict(srcDict)
        self.topics = srcDict["topics"]

    def isOffersAck(self):
        return True

class GxMessageFactory(object):
    def __init__(self, name="None"):
        self.myName = name

    def set_myName(self, newName):
        self.myName = newName

    def fromDict(self, srcDict):
        newMsg = GxBaseMsg()
        newMsg.fromDict(srcDict)

        if srcDict["TYPE"] == "REQUEST":
            newMsg = GxRequestMsg()
            newMsg.fromDict(srcDict)

        elif srcDict["TYPE"] == "SEND":
            newMsg = GxSendMsg()
            newMsg.fromDict(srcDict)
            
        elif srcDict["TYPE"] == "TXING":
            newMsg = GxTxingMsg()
            newMsg.fromDict(srcDict)

        elif srcDict["TYPE"] == "IHAVE":
            newMsg = GxIHaveMsg()
            newMsg.fromDict(srcDict)

        elif srcDict["TYPE"] == "DATA":
            newMsg = GxDataMsg()
            newMsg.fromDict(srcDict)

        elif srcDict["TYPE"] == "OFFERS_REQ":
            newMsg = GxOffersReqMsg()
            newMsg.fromDict(srcDict)

        elif srcDict["TYPE"] == "OFFERS_ACK":
            newMsg = GxOffersAckMsg()
            newMsg.fromDict(srcDict)

        return newMsg
                    

    def fromJSON(self, jsonString):
        toBeReturned = None
        try:
            toBeReturned = self.fromDict(json.loads(jsonString))
        except Exception as ex:
            print ex
        return toBeReturned

    def newRequestMsg(self):
        return GxRequestMsg(sender = self.myName)

    def newSendMsg(self):
        return GxSendMsg(sender = self.myName)
            
    def newTxingMsg(self):
        return GxTxingMsg(sender = self.myName)

    def newIHaveMsg(self):
        return GxIHaveMsg(sender = self.myName)

    def newDataMsg(self):
        return GxDataMsg(sender = self.myName)

    def newOffersReqMsg(self):
        return GxOffersReqMsg(sender = self.myName)

    def newOffersAckMsg(self):
        return GxOffersAckMsg(sender = self.myName)