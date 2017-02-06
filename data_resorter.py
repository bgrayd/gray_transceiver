import statistics
#def median(lst):
#    lst = sorted(lst)
#    if len(lst) < 1:
#            return None
#    if len(lst) %2 == 1:
#            return lst[((len(lst)+1)/2)-1]
#    else:
#            return float(sum(lst[(len(lst)/2)-1:(len(lst)/2)+1]))/2.0

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
        csvLine = csvLine.strip()
        names = csvLine.split(",")

        self.data["runTime"] = names[0]

        if names[1] == "json":
            self.data["jsonBool"] = True
        else:
            self.data["jsonBool"] = False

        self.data["computers"] = int(names[2])

        self.data["frequency"] = int(names[3])

        self.data["broadcastTopics"] = int(names[4])

        self.data["messageSize"] = int(names[5])
        
        for each in range(6, len(names)):
            if each != '':
                self.avgs.append(int(names[each]))


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
            line += str(each)
            line += ","

        line += "\n"

        return line

    def getAsCsvNoData(self):
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

        return line

    def getCsvHeader(self):
        return "runtime (seconds),serialization,number of computers,publish frequency (Hz),number of broadcast topics,message size (bytes),\n"

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

fileIn = open("throughput_test_avgs.csv", "r")

serialization = []
computers = []
frequencies = []
broadcastTopics = []
messageSizes = []

dataDict = {}

firstLine = True

for each in fileIn:
    if firstLine:
        firstLine = False
        continue
    newData = runSettings()
    newData.setFromCSV(each)
    
    if newData.getJsonBool() not in serialization:
        serialization.append(newData.getJsonBool())
    if newData.getComputers() not in computers:
        computers.append(newData.getComputers())
    if newData.getFrequency() not in frequencies:
        frequencies.append(newData.getFrequency())
    if newData.getBroadcastTopicNumber() not in broadcastTopics:
        broadcastTopics.append(newData.getBroadcastTopicNumber())
    if newData.getMessageSize() not in messageSizes:
        messageSizes.append(newData.getMessageSize())

    #another massive list, but this time going through a series of dicts
    if newData.getJsonBool() not in dataDict:
        dataDict[newData.getJsonBool()] = {}
    if newData.getComputers() not in dataDict[newData.getJsonBool()]:
        dataDict[newData.getJsonBool()][newData.getComputers()] = {}
    if newData.getFrequency() not in dataDict[newData.getJsonBool()][newData.getComputers()]:
        dataDict[newData.getJsonBool()][newData.getComputers()][newData.getFrequency()] = {}
    if newData.getBroadcastTopicNumber() not in dataDict[newData.getJsonBool()][newData.getComputers()][newData.getFrequency()]:
        dataDict[newData.getJsonBool()][newData.getComputers()][newData.getFrequency()][newData.getBroadcastTopicNumber()] = {}
    if newData.getMessageSize() not in dataDict[newData.getJsonBool()][newData.getComputers()][newData.getFrequency()][newData.getBroadcastTopicNumber()]:
        dataDict[newData.getJsonBool()][newData.getComputers()][newData.getFrequency()][newData.getBroadcastTopicNumber()][newData.getMessageSize()] = newData
    else:
        for each in newData.avgs:
            dataDict[newData.getJsonBool()][newData.getComputers()][newData.getFrequency()][newData.getBroadcastTopicNumber()][newData.getMessageSize()].avgs.append(each)

fileIn.close()


computers = sorted(computers)
frequencies = sorted(frequencies)
broadcastTopics = sorted(broadcastTopics)
messageSizes = sorted(messageSizes)

avgsSorted = open("throughput_test_avgs_sorted.csv", "w")
avgsAvged = open("throughput_test_avgs_avged.csv","w")
avgsMedian = open("throughput_test_avgs_median.csv", "w")
myLaptop = open("throughput_test_avg_myLaptop.csv","w")
netbook = open("throughput_test_avg_netbook.csv","w")

tempSettings = runSettings()
avgsSorted.write(tempSettings.getCsvHeader())
avgsAvged.write(tempSettings.getCsvHeader())
avgsMedian.write(tempSettings.getCsvHeader())
myLaptop.write(tempSettings.getCsvHeader())
netbook.write(tempSettings.getCsvHeader())

for serial in serialization:
    for comp in computers:
        for freq in frequencies:
            for broadcast in broadcastTopics:
                for msgSize in messageSizes:
                    try:
                        tempSettings = dataDict[serial][comp][freq][broadcast][msgSize]
                        #print(tempSettings.data)
                        #print(tempSettings.avgs)
                        avgsSorted.write(tempSettings.getAsCsv())
                        avgsAvged.write(tempSettings.getAsCsvNoData()+str( sum(tempSettings.avgs)*1.0/len(tempSettings.avgs) )+"\n")
                        myLaptop.write(tempSettings.getAsCsvNoData()+str(tempSettings.avgs[len(tempSettings.avgs)-1] )+"\n")
                        netbook.write(tempSettings.getAsCsvNoData()+str(tempSettings.avgs[0])+"\n")
                        avgsMedian.write(tempSettings.getAsCsvNoData()+str(statistics.median(tempSettings.avgs))+"\n")
                    except KeyError:
                        #print("KeyError")


avgsSorted.close()
avgsAvged.close()
avgsMedian.close()
myLaptop.close()
netbook.close()

computersAvg = open("throughput_computers_avg.csv","w")
computersMed = open("throughput_computers_med.csv","w")

computersAvg.write("Number of Computers:,")
computersMed.write("Number of Computers:,")

firstLine = ""
avgLines = {}
medLines = {}

for each in computers:
    #computersAvg.write(str(each)+",")
    #computersMed.write(str(each)+",")
    avgLines[each] = "\n"+str(each)+","
    medLines[each] = "\n"+str(each)+","

for serial in serialization:
    for freq in frequencies:
        for broadcast in broadcastTopics:
            for msgSize in messageSizes:
                lineName = ''
                if serial:
                    lineName += "json"
                else:
                    lineName += "ros"
                lineName += "-" + str(freq)
                lineName += "-" + str(broadcast)
                lineName += "-" + str(msgSize)

                #computersAvg.write("\n"+lineName+",")
                #computersMed.write("\n"+lineName+",")
                firstLine += lineName + ","

                for comp in computers:
                    try:
                        tempSettings = dataDict[serial][comp][freq][broadcast][msgSize]
                        #computersAvg.write(str( sum(tempSettings.avgs)*1.0/len(tempSettings.avgs) )+",")
                        #computersMed.write(str(statistics.median(tempSettings.avgs))+",")
                        avgLines[comp] += str( sum(tempSettings.avgs)*1.0/len(tempSettings.avgs) )+","
                        medLines[comp] += str(statistics.median(tempSettings.avgs))+","
                    except KeyError:
                        #computersAvg.write("NaN,")
                        #computersMed.write("NaN,")
                        avgLines[comp] += "-1,"
                        medLines[comp] += "-1,"
                        print("missing key in computer loops")

computersAvg.write(firstLine)
computersMed.write(firstLine)
for each in avgLines:
    computersAvg.write(avgLines[each])
    computersMed.write(medLines[each])

computersAvg.close()
computersMed.close()
