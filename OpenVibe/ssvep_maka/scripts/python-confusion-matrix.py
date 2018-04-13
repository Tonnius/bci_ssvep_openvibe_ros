import numpy as np
import os
from pandas_ml import ConfusionMatrix
import pickle
import collections
from sklearn.metrics import confusion_matrix

# import shutil
class MyOVBox(OVBox):
    def __init__(self):
        OVBox.__init__(self)
        self.actualLabels = []
        self.predictedLabels = []
        self.stimStartTime = 0
        self.stimEndTime = 0
        self.meanDetectTime = 0

        classDequeMaxLen = 1
        self.classThresh = 0.5
        self.currentLabelTimeStop = 0
        self.currentLabelTimeStart = 0
        self.currentLabel = -1
        self.actualLabelsProb = []
        self.predictedLabelsProb = []
        self.nrOfStimsClassified = 0
        self.nrOfStimsActual = 0

        self.classProbs = [collections.deque(maxlen=classDequeMaxLen), collections.deque(maxlen=classDequeMaxLen),
                           collections.deque(maxlen=classDequeMaxLen), collections.deque(maxlen=classDequeMaxLen)]

        self.predictedTime = 0
        self.newLabel = False
        self.meanDetectTimeProb = 0

        self.totalNrClassified = 0
        self.debugEnabled = False
        self.nothingEnabled = False
    def initialize(self):
        # nop
        return

    def getProbValue(self, inputNr, classNr):
        for chunkIndexMatrix in range(len(self.input[inputNr])):
            probMatrix = self.input[inputNr].pop() #probMatrix[0] is stimulated, probMatrix[1] is non-stimulated
            self.classProbs[classNr].append([probMatrix[0], self.getCurrentTime()])
            if probMatrix[0] > self.classThresh:
                return True

        return False

    def process(self):
        # stim = 0
        # for chunkIndex in range(len(self.input[0])):
        #     chunk = self.input[0].pop()
        #     if type(chunk) == OVStimulationSet:
        #         for stimIdx in range(len(chunk)):
        #             stim = chunk.pop()
        #             #print 'Received stim on input0', stim.identifier-33025, 'stamped at', stim.date, 's'
        #             self.actualLabels.append(stim.identifier-33025)
        #
        # for chunkIndex1 in range(len(self.input[1])):
        #     chunk1 = self.input[1].pop()
        #     if type(chunk1) == OVStimulationSet:
        #         for stimIdx1 in range(len(chunk1)):
        #             stim1 = chunk1.pop()
        #             # if stim is not 0:
        #             # if stim1.date > stim.date:
        #             # print 'Received stim on input1', stim1.identifier-33025, 'stamped at', stim1.date, 's'
        #             self.predictedLabels.append(stim1.identifier-33025)
        #
        # for chunkIndex2 in range(len(self.input[2])):
        #     chunk2 = self.input[2].pop()
        #     if type(chunk2) == OVStimulationSet:
        #         for stimIdx2 in range(len(chunk2)):
        #             stim2 = chunk2.pop()
        #             if (stim2.identifier == 33054):
        #                 self.stimStartTime = stim2.date
        #             elif (stim2.identifier == 33055):
        #                 self.stimEndTime = stim2.date
        #                 if (self.stimEndTime > self.stimStartTime):
        #                     self.meanDetectTime += self.stimEndTime - self.stimStartTime
        #                 else:
        #                     print "Start Time came after End Time. Something went wrong."

        for chunkIndex in range(len(self.input[7])):
            chunk = self.input[7].pop()
            if type(chunk) == OVStimulationSet:
                for stimIdx in range(len(chunk)):
                    stim = chunk.pop()
                    #print 'Received stim on input0', stim.identifier-33025, 'stamped at', stim.date, 's'
                    if(stim.identifier > 33024 and stim.identifier < 33030):
                        self.currentLabel = stim.identifier-33024
                        self.currentLabelTimeStart = stim.date + 1.0
                        self.currentLabelTimeStop = self.currentLabelTimeStart + 7.0
                        self.newLabel = True
                        self.nrOfStimsActual += 1
                        if self.debugEnabled:
                            print 'Total classified up to now py: '+str(self.totalNrClassified)
        probsAll = [0.0, 0.0, 0.0, 0.0]
        if (self.getCurrentTime() > self.currentLabelTimeStop) and self.newLabel:
            self.newLabel = False
            if self.debugEnabled:
                print "didnt classify!"

            if self.nothingEnabled and (self.currentLabel is not -1):
                for i in range(4):
                    for prob in self.classProbs[i]:
                        probsAll[i] += prob[0]

                maxpos = probsAll.index(max(probsAll))

                self.actualLabelsProb.append(self.currentLabel)
                self.predictedLabelsProb.append(maxpos + 1)
                self.totalNrClassified += 1
                self.nrOfStimsClassified += 1
                self.meanDetectTimeProb += 7




        classHit = [self.getProbValue(inputNr=3, classNr=0),
                    self.getProbValue(inputNr=4, classNr=1),
                    self.getProbValue(inputNr=5, classNr=2),
                    self.getProbValue(inputNr=6, classNr=3)]


        if True in classHit:
            for i in range(4):
                for prob in self.classProbs[i]:
                    #if prob[1] > (self.getCurrentTime()-1): #only use probabilities from last second
                    if prob[0] > self.classThresh:
                        self.predictedTime = self.getCurrentTime()
                        probsAll[i] += prob[0]
            if self.debugEnabled:
                print probsAll

            maxpos = probsAll.index(max(probsAll))
            #
            if (self.currentLabel is not -1) and \
                    (np.count_nonzero(probsAll) == 1) and \
                    (self.currentLabelTimeStart <= self.predictedTime) and \
                    (self.predictedTime <= self.currentLabelTimeStop):
                self.actualLabelsProb.append(self.currentLabel)
                self.predictedLabelsProb.append(maxpos+1)
                self.totalNrClassified += 1
                if self.newLabel and (self.predictedTime - self.currentLabelTimeStart) > 0.5: #so that old classifications dont interrupt
                    self.meanDetectTimeProb += self.predictedTime - self.currentLabelTimeStart
                    self.newLabel = False
                    self.nrOfStimsClassified += 1
                    if self.debugEnabled:
                        print "nr of stims is " + str(self.nrOfStimsClassified)

                    print "predicted class was "+str(maxpos+1) + " label was: "+str(self.currentLabel)
        # else:
        #	print 'Received chunk of type ', type(chunk), " looking for StimulationSet"
        return

    def uninitialize(self):
        self.meanDetectTimeProb /= self.nrOfStimsClassified
        print 'nr of stims classified '+str(self.nrOfStimsClassified)+' of all '+str(self.nrOfStimsActual)
        #print 'size of predicted ' + str(len(self.predictedLabelsProb))+ " size of actual "+str(len(self.actualLabelsProb))
        #print "mean time old: "+str(self.meanDetectTime / self.nrOfStimsClassified)
        print "mean time new: "+str(self.meanDetectTimeProb)
        #confusion_matrix = ConfusionMatrix(self.actualLabelsProb, self.predictedLabelsProb)
        cmSklearn = confusion_matrix(self.actualLabelsProb, self.predictedLabelsProb)
        print cmSklearn
        cmSklearn = cmSklearn.astype('float') / cmSklearn.sum(axis=1)[:, np.newaxis]
        print cmSklearn
        #print("Confusion matrix:\n%s" % confusion_matrix)
        #confusion_matrix.print_stats()
        nrOfSubj = int(self.setting['Nr of subjects'])

        dirToWrite = '/home/tonnius/Git/magister_BCI/OpenVibe/ssvep_maka/data/'
        #dataDirFileNames = sorted(os.listdir(dirToWrite))
        #print dataDirFileNames
        fileNrToWrite = self.setting['Current Subject Nr']

        exSettings = {'ch': self.setting['channels'],
                      'epDur': self.setting['Epoch Duration'],
                      'epInt': self.setting['Epoch Interval'],
                      'freqTol': self.setting['Freq Tol'],
                      'simFreq': self.setting['SimulationFreq'],
                      'nrOfSubjects': nrOfSubj,
                      'currentSubjNr': fileNrToWrite}

        #if int(fileNrToWrite) > (nrOfSubj - 1):
        #   fileNrToWrite = '0'

        settingsFileName = ''
        for key,value in exSettings.items():
            if(key != 'simFreq'):
                settingsFileName += str(value)[2:]

        writeFile = dirToWrite + 'Ex' + settingsFileName + 'subject' + str(fileNrToWrite)
        data = {'settings': exSettings,
                  'actual': self.actualLabelsProb,
                  'predicted': self.predictedLabelsProb,
                  'detectTime': self.meanDetectTimeProb,
                  'stims': {'stimsNrClassified': self.nrOfStimsClassified, 'stimsNrActual': self.nrOfStimsActual}
                  }

        pickle.dump(data, open(writeFile, 'wb'))

        #os.rename(dirToWrite + dataDirFileNames[0], dirToWrite + str(int(fileNrToWrite) + 1))

        # with open(dirToWrite, 'a') as f:
        # np.savetxt(f, (self.actualLabels, self.predictedLabels))
        #print "meanDetectTime " + str(self.meanDetectTimeProb) + "and nr of stims " + str(self.nrOfStimsClassified)
        #print "Mean detect time was " + str(self.meanDetectTime)
        print 'File saved to ' + writeFile
        #print 'actual labels array size: ', len(self.actualLabels), ' predictedLabels array size: ', len(self.predictedLabels)



box = MyOVBox()
