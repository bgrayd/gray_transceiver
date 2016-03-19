#!/usr/bin/env python

import json


class GxBaseMsg(object):
    def __init__(self, sender, msgType):
        self.sender = sender
        self.msgType = msgType

    def toDict(self):
        return {"TYPE":self.msgType, "SENDER":self.sender}

    def toJSON(self):
        return json.dumps(self.toDict())

    def isIAm(self):
        return False 

    def isNew(self):
        return False

    def isDeny(self):
        return False

    def isAccept(self):
        return False

    def isRequest(self):
        return False

    def isSend(self):
        return False

    def isIHave(self):
        return False

    def isTxing(self):
        return False

    def isPortPoll(self):
        return False

    def isPortTaken(self):
        return False

    def isOffersReq(self):
        return False

    def isOffersAck(self):
        return False

    def isData(self):
        return False

class GxIAmMsg(GxBaseMsg):
    def __init__(self, sender):
        super(GxIAmMsg).__init__(sender, "IAM")

    def isIAm(self):
        return True

class GxNewAcptDenyMsg(GxBaseMsg):
    def __init__(self, sender, msgType, nameReq, uId):
        super(GxNewAcptDenyMsg).__init__(sender, msgType)
        self.nameReq = nameReq
        self.uId = uId

    def toDict(self):
        toBeReturned = super(GxNewMsg).toDict()
        toBeReturned["name requested"] = self.nameReq
        toBeReturned["unique id"] = self.uId
        return toBeReturned

class GxNewMsg(GxNewAcptDenyMsg):
    def __init__(self, nameReq, uId):
        super(GxNewMsg).__init__("NULL", "NEW", nameReq, uId)

    def isNew(self):
        return True
        
class GxDenyMsg(GxNewAcptDenyMsg):
    def __init__(self, sender, nameReq, uId):
        super(GxDenyMsg).__init__(sender, "DENY", nameReq, uId)

    def isDeny(self):
        return True

class GxAcceptMsg(GxNewAcptDenyMsg):
    def __init__(self, sender, nameReq, uId):
        super(GxAcceptMsg).__init__(sender, "ACCEPT", nameReq, uId)

    def isAccept(self):
        return True

class GxRequestMsg(GxBaseMsg):
    def __init__(self, sender, description, rosMsgType):
        super(GxRequestMsg).__init__(sender, "REQUEST")
        self.description = description
        self.rosMsgType = rosMsgType

    def toDict(self):
        toBeReturned = super(GxRequestMsg).toDict()
        toBeReturned["description"] = self.description
        toBeReturned["message type"] = self.rosMsgType
        return toBeReturned

    def isRequest(self):
        return True

class GxSendMsg(GxBaseMsg):
    def isSend(self):
        return True

class GxIHaveMsg(GxBaseMsg):
    def isIHave(self):
        return True

class GxTxingMsg(GxBaseMsg):
    def isTxing(self):
        return True

class GxPortMsg(GxBaseMsg):
    pass
class GxPortPollMsg(GxPortMsg):
    def isPortPoll(self):
        return True

class GxPortTakenMsg(GxPortMsg):
    def isPortTaken(self):
        return True

class GxOffersReqMsg(GxBaseMsg):
    def __init__(self, sender):
        super(GxOffersRegMsg).__init__(sender, "OFFERS_REQ")

    def isOffersReq(self):
        return True

class GxOffersAckMsg(GxBaseMsg):
    def isOffersAck(self):
        return True

class GxDataMsg(GxBaseMsg):
    def isData(self):
        return True

class GxMessageFactory(object):
    def __init__(self, name="NULL"):
        self.myName = name

    def set_myName(self, newName):
        self.myName = newName

    def fromDict(self, srcDict):
        pass

    def fromJSON(self, jsonString):
        return self.from_dict(json.loads(jsonString))