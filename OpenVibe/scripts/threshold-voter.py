import collections

class MyOVBox(OVBox):
    def __init__(self):
        OVBox.__init__(self)
        classDequeMaxLen = 1
        self.classProbs = [collections.deque(maxlen=classDequeMaxLen),
                           collections.deque(maxlen=classDequeMaxLen),
                           collections.deque(maxlen=classDequeMaxLen),
                           collections.deque(maxlen=classDequeMaxLen)]

        self.debugEnabled = False

    def initialize(self):
        threshString = self.setting['Thresholds']  # Get thresholds
        threshs = threshString.split(':')
        if threshs:
            self.classThresh = float(threshs[0])
            self.maxProbDiffThresh = float(threshs[1])
        else:
            self.classThresh = 0.5
            self.maxProbDiffThresh = 0.25

    # Get classification probability
    def getProbValue(self, inputNr, classNr):
        for chunkIndexMatrix in range(len(self.input[inputNr])):
            probMatrix = self.input[inputNr].pop()  # probMatrix[0] is stimulated, probMatrix[1] is non-stimulated
            self.classProbs[classNr].append([probMatrix[0], self.getCurrentTime()])
            if probMatrix[0] > self.classThresh:
                return True

        return False

    # Process loop for classification
    def process(self):
        probsAll = [0.0, 0.0, 0.0, 0.0]

        classHit = [self.getProbValue(inputNr=3, classNr=0),
                    self.getProbValue(inputNr=4, classNr=1),
                    self.getProbValue(inputNr=5, classNr=2),
                    self.getProbValue(inputNr=6, classNr=3)]

        if True in classHit:  # Clear first threshold
            for i in range(4):
                for prob in self.classProbs[i]:
                    if prob[0] > self.classThresh:
                        probsAll[i] += prob[0]

            if self.debugEnabled:
                print "probsAll: " + str(probsAll)

            maxProb = max(probsAll)
            maxpos = probsAll.index(maxProb)
            del probsAll[maxpos]
            maxProb2 = max(probsAll)
            maxPosDif = maxProb - maxProb2

            if self.debugEnabled:
                print "maxPosDif: " + str(maxPosDif) + "maxpos: " + str(maxpos)

            if (maxPosDif >= self.maxProbDiffThresh):  # Clear second threshold
                if self.debugEnabled:
                    print "predicted class was " + str(maxpos + 1)
                stimSet = OVStimulationSet(self.getCurrentTime(), self.getCurrentTime() + 1. / self.getClock())
                stimSet.append(OVStimulation(33025 + maxpos, self.getCurrentTime(), 0.))
                self.output[0].append(stimSet)  # Send stimulation to TCP Writer
        return

    def uninitialize(self):
        return

box = MyOVBox()
