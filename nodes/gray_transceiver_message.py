#!/usr/bin/env python

import json

from gray_transceiver.msg import GxTopicMetaInformation


class GxBaseMsg(object):
    def __init__(self, sender, msgType):
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
        super(GxTopicMetaInfoMsg, self).__init__(sender, "REQUEST")
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
        toBeReturned["data"] = self.data
        return toBeReturned

    def fromDict(self, srcDict):
        super(GxDataMsg, self).fromDict(srcDict)
        self.data = srcDict["data"]

    def setData(self, newData):
        self.data = newData

    def getData(self):
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
        except:
            pass
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

