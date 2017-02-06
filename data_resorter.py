#! python2.7

class runSettings(object):
    def __init__(self):
        self.data = {}

        #default values
        self.data["computers"] = 0
        self.data["frequency"] = 10
        self.data["messageSize"] = 10
        self.data["broadcastTopics"] = 1
        self.data["jsonBool"] = False
        self.data["runTime"] = 300 #5 minutes

		self.avgs = []

    def load(self, newData):
        self.data = json.loads(newData)
    
    def dump(self):
        return json.dumps(self.data)

    def setFromCSV(self, csvLine):
        names = csvLine.split(",")

        self.data["runTime"] = names[0]

        if names[1] == "json":
            self.data["jsonBool"] = True
        else:
            self.data["jsonBool"] = False

        self.data["computers"] = names[2]

        self.data["frequency"] = names[3]

        self.data["broadcastTopics"] = names[4]

        self.data["messageSize"] = names[5]

		for each in range(6, len(names)):
			self.avgs.append(names[each])


    def getAsCsv(self):
        line = ""

        line += str(self.data["runTime"])
        line += ","
        
        if self.data["jsonBool"]:
            line += "json,"
        else:
            line += "ros,"

        line += str(self.data["computers"])
        line += ","

        line += str(self.data["frequency"])
        line += ","

        line += str(self.data["broadcastTopics"])
        line += ","

        line += str(self.data["messageSize"])
		line += ","

		for each in self.avgs:
			line += str(self.avgs)
			line += ","

		line += "\n"

        return line

    def getCsvHeader(self):
        return "runtime (seconds),serialization,number of computers,publish frequency (Hz),number of broadcast topics,message size (bytes),"

    def setComputers(self, number):
        self.data["computers"] = number

    def getComputers(self):
        return self.data["computers"]

    def setFrequency(self, freq):
        self.data["frequency"] = freq

    def getFrequency(self):
        return self.data["frequency"]

    def setMessageSize(self, msgSize):
        self.data["messageSize"] = msgSize

    def getMessageSize(self):
        return self.data["messageSize"]

    def setBroadcastTopicNumber(self, number):
        self.data["broadcastTopics"] = number

    def getBroadcastTopicNumber(self):
        return self.data["broadcastTopics"]

    def setJsonBool(self, jsonBool):
        self.data["jsonBool"] = jsonBool

    def getJsonBool(self):
        return self.data["jsonBool"]

    def setRunTime(self, runTime):
        self.data["runTime"] = runTime

    def getRunTime(self):
        return self.data["runTime"]