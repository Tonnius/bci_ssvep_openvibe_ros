#import numpy as np
import os
# from pandas_ml import ConfusionMatrix
import pickle


# import shutil
class MyOVBox(OVBox):
    def __init__(self):
        OVBox.__init__(self)
        self.actualLabels = []
        self.predictedLabels = []
        self.nrOfStims = 0
        self.stimStartTime = 0
        self.stimEndTime = 0
        self.meanDetectTime = 0
    def initialize(self):
        # nop
        return

    def process(self):
        stim = 0
        for chunkIndex in range(len(self.input[0])):
            chunk = self.input[0].pop()
            if type(chunk) == OVStimulationSet:
                for stimIdx in range(len(chunk)):
                    stim = chunk.pop()
                    # print 'Received stim on input0', stim.identifier, 'stamped at', stim.date, 's'
                    self.actualLabels.append(stim.identifier)

        for chunkIndex1 in range(len(self.input[1])):
            chunk1 = self.input[1].pop()
            if type(chunk1) == OVStimulationSet:
                for stimIdx1 in range(len(chunk1)):
                    stim1 = chunk1.pop()
                    # if stim is not 0:
                    # if stim1.date > stim.date:
                    # print 'Received stim on input1', stim1.identifier, 'stamped at', stim1.date, 's'
                    self.predictedLabels.append(stim1.identifier)

        for chunkIndex2 in range(len(self.input[2])):
            chunk2 = self.input[2].pop()
            if type(chunk2) == OVStimulationSet:
                for stimIdx2 in range(len(chunk2)):
                    stim2 = chunk2.pop()
                    # if stim is not 0:
                    # if stim1.date > stim.date:
                    #print 'Received stim on input2', stim2.identifier, 'stamped at', stim2.date, 's'
                    #stimCode = OpenViBE_stimulation[stim2.identifier]
                    if (stim2.identifier == 33054):
                        self.stimStartTime = stim2.date
                        self.nrOfStims += 1
                    elif (stim2.identifier == 33055):
                        self.stimEndTime = stim2.date
                        if (self.stimEndTime > self.stimStartTime):
                            self.meanDetectTime += self.stimEndTime - self.stimStartTime
                        else:
                            print "Start Time came after End Time. Something went wrong."
        #for chunkIndexMatrix in range(len(self.input[3])):
        #    chunk = self.input[3].pop()
         #   print chunk
          #  for matrix in range(len(chunk)):

           #     thing = chunk.pop()
                #print thing
                    
        # else:
        #	print 'Received chunk of type ', type(chunk), " looking for StimulationSet"
        return

    def uninitialize(self):

        nrOfSubj = int(self.setting['Nr of subjects'])

        dirToWrite = '/home/tonnius/Git/magister_BCI/OpenVibe/ssvep_maka/data/'
        dataDirFileNames = sorted(os.listdir(dirToWrite))
        #print dataDirFileNames
        fileNrToWrite = dataDirFileNames[0]
        self.meanDetectTime /= self.nrOfStims
        exSettings = {'ch': self.setting['channels'],
                      'epDur': self.setting['Epoch Duration'],
                      'epInt': self.setting['Epoch Interval'],
                      'freqTol': self.setting['Freq Tol'],
                      'nrOfSubjects': nrOfSubj}

        #print 'nr of subjects ' + str(nrOfSubj)
        if int(fileNrToWrite) > (nrOfSubj - 1):
            fileNrToWrite = '0'

        settingsFileName = ''
        for key,value in exSettings.items():
            settingsFileName += str(value)[2:]

        writeFile = dirToWrite + 'Ex' + settingsFileName + 'subject' + fileNrToWrite
        labels = {'settings': exSettings,
                  'actual': self.actualLabels,
                  'predicted': self.predictedLabels,
                  'detectTime': self.meanDetectTime}

        pickle.dump(labels, open(writeFile, 'wb'))

        os.rename(dirToWrite + dataDirFileNames[0], dirToWrite + str(int(fileNrToWrite) + 1))

        # with open(dirToWrite, 'a') as f:
        # np.savetxt(f, (self.actualLabels, self.predictedLabels))
        print "meanDetectTime " + str(self.meanDetectTime) + "and nr of stims " + str(self.nrOfStims)
        print "Mean detect time was " + str(self.meanDetectTime)
        print 'File saved to ' + writeFile
        print 'actual labels array size: ', len(self.actualLabels), ' predictedLabels array size: ', len(
            self.predictedLabels)
    # confusion_matrix = ConfusionMatrix(self.actualLabels, self.predictedLabels)
    # print("Confusion matrix:\n%s" % confusion_matrix)
    # confusion_matrix.print_stats()


box = MyOVBox()
