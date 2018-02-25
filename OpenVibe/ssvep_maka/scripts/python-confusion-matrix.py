import numpy as np
import os
# from pandas_ml import ConfusionMatrix
import pickle


# import shutil
class MyOVBox(OVBox):
    def __init__(self):
        OVBox.__init__(self)
        self.actualLabels = []
        self.predictedLabels = []

    def initialize(self):
        # nop
        return

    def process(self):
        stim = 0
        for chunkIndex in range(len(self.input[0])):
            chunk = self.input[0].pop()
            if (type(chunk) == OVStimulationSet):
                for stimIdx in range(len(chunk)):
                    stim = chunk.pop()
                    # print 'Received stim on input0', stim.identifier, 'stamped at', stim.date, 's'
                    self.actualLabels.append(stim.identifier)

        for chunkIndex1 in range(len(self.input[1])):
            chunk1 = self.input[1].pop()
            if (type(chunk1) == OVStimulationSet):
                for stimIdx1 in range(len(chunk1)):
                    stim1 = chunk1.pop()
                    # if stim is not 0:
                    # if stim1.date > stim.date:
                    # print 'Received stim on input1', stim1.identifier, 'stamped at', stim1.date, 's'
                    self.predictedLabels.append(stim1.identifier)

        # else:
        #	print 'Received chunk of type ', type(chunk), " looking for StimulationSet"
        return

    def uninitialize(self):
        nrOfSubjects = 3
        dirToWrite = '/home/tonnius/Git/magister_BCI/OpenVibe/ssvep_maka/data/'
        dataDirFileNames = sorted(os.listdir(dirToWrite))
        print dataDirFileNames
        fileNrToWrite = dataDirFileNames[0]
        exSettings = [self.setting['channels'],
                      self.setting['Epoch Duration'],
                      self.setting['Epoch Interval'],
                      self.setting['Freq Tol']]

        if int(fileNrToWrite) > nrOfSubjects:
            fileNrToWrite = '1'

        settingsFileName = ''
        for setting in exSettings:
            settingsFileName = settingsFileName + str(setting)[2:]

        writeFile = dirToWrite + 'Ex' + settingsFileName + 'subject' + fileNrToWrite
        labels = []

        print 'Experiment setting: '.join([str(item) for item in exSettings])
        labels.append(exSettings)
        labels.append(self.actualLabels)
        labels.append(self.predictedLabels)
        pickle.dump(labels, open(writeFile, 'wb'))

        os.rename(dirToWrite + dataDirFileNames[0], dirToWrite + str(int(fileNrToWrite) + 1))

        # with open(dirToWrite, 'a') as f:
        # np.savetxt(f, (self.actualLabels, self.predictedLabels))
        print 'File saved to ' + writeFile
        print 'actual labels array size: ', len(self.actualLabels), ' predictedLabels array size: ', len(
            self.predictedLabels)
    # confusion_matrix = ConfusionMatrix(self.actualLabels, self.predictedLabels)
    # print("Confusion matrix:\n%s" % confusion_matrix)
    # confusion_matrix.print_stats()


box = MyOVBox()
