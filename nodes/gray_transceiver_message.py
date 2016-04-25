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
        return self.msgTypef

    def fromDict(self, srcDict):
        self.sender = srcDict["SENDER"]
        self.msgType = srcDict["TYPE"]

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
    def __init__(self, sender=NULL):
        super(GxIAmMsg).__init__(sender, "IAM")

    def fromDict(self, srcDict):
        super(GxIAmMsg).fromDict(srcDict)

    def isIAm(self):
        return True

class GxNewAcptDenyMsg(GxBaseMsg):
    def __init__(self, sender=NULL, msgType=NULL, nameReq=NULL, uId=NULL):
        super(GxNewAcptDenyMsg).__init__(sender, msgType)
        self.nameReq = nameReq
        self.uId = uId

    def fromDict(self, srcDict):
        super(GxNewAcptDenyMsg).fromDict(srcDict)
        self.nameReq = srcDict["name_requested"]
        self.uId = srcDict["unique_id"]

    def toDict(self):
        toBeReturned = super(GxNewMsg).toDict()
        toBeReturned["name_requested"] = self.nameReq
        toBeReturned["unique_id"] = self.uId
        return toBeReturned

    def setNameRequested(self, nameReq):
        self.nameReq = nameReq

    def getNameRequested(self):
        return self.nameReq

    def setUniqueId(self, uId):
        self.uId = uId

    def getUniqueId(self):
        return uId

class GxNewMsg(GxNewAcptDenyMsg):
    def __init__(self, nameReq=NULL, uId=NULL):
        super(GxNewMsg).__init__("NULL", "NEW", nameReq, uId)

    def isNew(self):
        return True
        
class GxDenyMsg(GxNewAcptDenyMsg):
    def __init__(self, sender=NULL, nameReq=NULL, uId=NULL):
        super(GxDenyMsg).__init__(sender, "DENY", nameReq, uId)

    def isDeny(self):
        return True

class GxAcceptMsg(GxNewAcptDenyMsg):
    def __init__(self, sender=NULL, nameReq=NULL, uId=NULL):
        super(GxAcceptMsg).__init__(sender, "ACCEPT", nameReq, uId)

    def isAccept(self):
        return True

class GxRequestMsg(GxBaseMsg):
    def __init__(self, sender=NULL, description=NULL, rosMsgType=NULL):
        super(GxRequestMsg).__init__(sender, "REQUEST")
        self.description = description
        self.rosMsgType = rosMsgType

    def toDict(self):
        toBeReturned = super(GxRequestMsg).toDict()
        toBeReturned["description"] = self.description
        toBeReturned["message_type"] = self.rosMsgType
        return toBeReturned

    def fromDict(self, srcDict):
        super(GxRequestMsg).fromDict(srcDict)
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
    def __init__(self, sender=NULL):
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
        newMsg = GxBaseMsg()
        newMsg.fromDict(srcDict)

        if srcDict["TYPE"] == "NEW":
            newMsg = GxNewMsg()
            newMsg.fromDict(srcDict)

        elif srcDict["TYPE"] == "ACCEPT":
            newMsg = GxAcceptMsg()
            newMsg.fromDict(srcDict)

        elif srcDict["TYPE"] == "DENY":
            newMsg = GxDenyMsg()
            newMsg.fromDict(srcDict)
    
        elif srcDict["TYPE"] == "IAM":
            newMsg = GxIAmMsg()
            newMsg.fromDict(srcDict)

        elif srcDict["TYPE"] == "REQUEST":
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
        
        elif srcDict["TYPE"] == "PORT_POLL":
            newMsg = GxPortPollMsg()
            newMsg.fromDict(srcDict)

        elif srcDict["TYPE"] == "PORT_TAKEN":
            newMsg = GxPortTakenMsg()
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
