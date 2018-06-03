import numpy as np
import pickle
import collections
from sklearn.metrics import confusion_matrix

class MyOVBox(OVBox):
    def __init__(self):
        OVBox.__init__(self)
        self.currentLabelTimeStop = 0
        self.currentLabelTimeStart = 0
        self.currentLabel = -1
        self.actualLabelsProb = []
        self.predictedLabelsProb = []
        self.nrOfStimsClassified = 0
        self.nrOfStimsActual = 0

        classDequeMaxLen = 1
        self.classProbs = [collections.deque(maxlen=classDequeMaxLen),
                           collections.deque(maxlen=classDequeMaxLen),
                           collections.deque(maxlen=classDequeMaxLen),
                           collections.deque(maxlen=classDequeMaxLen)]

        self.predictedTime = 0
        self.newLabel = False
        self.meanDetectTimeNewStim = 0

        self.totalNrClassified = 0
        self.debugEnabled = False
        self.predNothingEnabled = False
        self.notClassified = 0

        self.classThresh = 0.5
        self.maxProbDiffThresh = 0.25

    def initialize(self):
        threshString = self.setting['Thresholds']  # Get thresholds
        threshs = threshString.split(':')
        if threshs:
            self.classThresh = float(threshs[0])
            self.maxProbDiffThresh = float(threshs[1])

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
        for chunkIndex in range(len(self.input[7])):  # Get actual stimuli value
            chunk = self.input[7].pop()
            if type(chunk) == OVStimulationSet:
                for stimIdx in range(len(chunk)):
                    stim = chunk.pop()
                    if self.debugEnabled:
                        print 'Received stim on input 7', stim.identifier - 33025, 'stamped at', stim.date, 's'
                    if (stim.identifier > 33024 and stim.identifier < 33030):
                        self.currentLabel = stim.identifier - 33024
                        self.currentLabelTimeStart = stim.date + 1.0
                        self.currentLabelTimeStop = self.currentLabelTimeStart + 7.0
                        self.newLabel = True
                        self.nrOfStimsActual += 1
                        if self.debugEnabled:
                            print 'Total classified up to now: ' + str(self.totalNrClassified)

        probsAll = [0.0, 0.0, 0.0, 0.0]

        if (self.getCurrentTime() > self.currentLabelTimeStop) and self.newLabel:  # Mark as unclassified if not classified in allotted time
            self.newLabel = False
            self.notClassified += 1
            if self.debugEnabled:
                print "didnt classify!"

            if self.predNothingEnabled and (self.currentLabel is not -1):  # Predict best guess if out of time
                for i in range(4):
                    for prob in self.classProbs[i]:
                        probsAll[i] += prob[0]

                maxpos = probsAll.index(max(probsAll))

                self.actualLabelsProb.append(self.currentLabel)
                self.predictedLabelsProb.append(maxpos + 1)
                self.totalNrClassified += 1
                self.nrOfStimsClassified += 1
                self.meanDetectTimeNewStim += 7

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

            self.predictedTime = self.getCurrentTime()
            if (self.currentLabel is not -1) and \
                    maxPosDif >= self.maxProbDiffThresh and \
                    (self.currentLabelTimeStart <= self.predictedTime) and \
                    (self.predictedTime <= self.currentLabelTimeStop):  # Clear second threshold

                self.actualLabelsProb.append(self.currentLabel)
                self.predictedLabelsProb.append(maxpos + 1)
                self.totalNrClassified += 1
                meanTimeTemp = self.predictedTime - self.currentLabelTimeStart
                if self.newLabel:
                    self.meanDetectTimeNewStim += meanTimeTemp
                    self.newLabel = False
                    self.nrOfStimsClassified += 1
                    if self.debugEnabled:
                        print "nr of stims is " + str(self.nrOfStimsClassified)
                        print "predicted class was " + str(maxpos + 1) + " label was: " + str(self.currentLabel)
        return

    # Write results to a file
    def uninitialize(self):
        self.meanDetectTimeNewStim /= self.nrOfStimsClassified
        print "mean time for first classification (new stimulus): " + str(self.meanDetectTimeNewStim)
        cmSklearn = confusion_matrix(self.actualLabelsProb, self.predictedLabelsProb)
        cmSklearn = cmSklearn.astype('float') / cmSklearn.sum(axis=1)[:, np.newaxis]
        print "Confusion matrix:"
        print cmSklearn

        nrOfSubj = int(self.setting['Nr of subjects'])
        dirToWrite = self.setting['Results Directory']
        fileNrToWrite = self.setting['Current Subject Nr']

        exSettings = {'ch': self.setting['channels'],
                      'epDur': self.setting['Epoch Duration'],
                      'epInt': self.setting['Epoch Interval'],
                      'freqTol': self.setting['Freq Tol'],
                      'simFreq': self.setting['SimulationFreq'],
                      'nrOfSubjects': nrOfSubj,
                      'currentSubjNr': fileNrToWrite,
                      'classTresh': self.classThresh,
                      'maxProbDiffThresh': self.maxProbDiffThresh}

        settingsFileName = ''
        for key, value in exSettings.items():
            if (key != 'simFreq'):
                settingsFileName += str(value)[2:]

        writeFile = dirToWrite + 'Ex' + settingsFileName + 'subject' + str(fileNrToWrite)
        data = {'settings': exSettings,
                'actual': self.actualLabelsProb,
                'predicted': self.predictedLabelsProb,
                'detectTime': self.meanDetectTimeNewStim,
                'stims': {'stimsNrClassified': self.nrOfStimsClassified,
                          'stimsNrActual': self.nrOfStimsActual,
                          'totalNrClassified': self.totalNrClassified,
                          'totalTime': self.nrOfStimsActual * 7,
                          'notClassified': self.notClassified}
                }

        pickle.dump(data, open(writeFile, 'wb'))
        print 'File saved to ' + writeFile

box = MyOVBox()
