#!/usr/bin/env python

import json


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

class GxMetaTopicInfoMsg(GxBaseMsg):
    def __init__(self, sender=None, msgType=None, description=None, rosMsgType=None):
        super(GxMetaTopicInfoMsg).__init__(sender, "REQUEST")
        self.description = description
        self.rosMsgType = rosMsgType

    def toDict(self):
        toBeReturned = super(GxMetaTopicInfoMsg).toDict()
        toBeReturned["description"] = self.description
        toBeReturned["message_type"] = self.rosMsgType
        return toBeReturned

    def fromDict(self, srcDict):
        super(GxMetaTopicInfoMsg).fromDict(srcDict)
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

class GxRequestMsg(GxMetaTopicInfoMsg):
    def __init__(self, sender=None, description=None, rosMsgType=None):
        super(GxRequestMsg).__init__(sender, "REQUEST", description, rosMsgType)

    def isRequest(self):
        return True

class GxSendMsg(GxMetaTopicInfoMsg):
    def __init__(self, sender=None, description=None, rosMsgType=None):
        super(GxSendMsg).__init__(sender, "SEND", description, rosMsgType)
    def isSend(self):
        return True

class GxIHaveMsg(GxMetaTopicInfoMsg):
    def __init__(self, sender=None, description=None, rosMsgType=None):
        super(GxIHaveMsg).__init__(sender, "IHAVE", description, rosMsgType)
    def isIHave(self):
        return True

class GxTxingMsg(GxMetaTopicInfoMsg):
    def __init__(self, sender=None, description=None, rosMsgType=None):
        super(GxTxingMsg).__init__(sender, "TXING", description, rosMsgType)
    def isTxing(self):
        return True

class GxOffersReqMsg(GxBaseMsg):
    def __init__(self, sender=None):
        super(GxOffersRegMsg).__init__(sender, "OFFERS_REQ")

    def isOffersReq(self):
        return True

class GxOffersAckMsg(GxBaseMsg):
    def __init__(self, sender=None):
        super(GxOffersAckMsg).__init__(sender, "OFFERS_ACK")
    def isOffersAck(self):
        return True

class GxDataMsg(GxBaseMsg):
    def __init__(self, sender=None):
        super(GxDataMsg).__init__(sender, "DATA")
    def isData(self):
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

        elif srcDict["TYPE"] == "OFFERS_REQ":
            newMsg = GxOffersReqMsg()
            newMsg.fromDict(srcDict)

        elif srcDict["TYPE"] == "OFFERS_ACK":
            newMsg = GxOffersAckMsg()
            newMsg.fromDict(srcDict)

        return newMsg
                    

    def fromJSON(self, jsonString):
        return self.fromDict(json.loads(jsonString))
