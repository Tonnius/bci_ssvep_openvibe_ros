import pickle
import os
import numpy as np
import math
from sklearn.metrics import confusion_matrix
from sklearn.metrics import classification_report


class MaxRes:
    def __init__(self):
        self.maxCmSklearn = None
        self.maxItr = 0
        self.maxItrFileN = ''
        self.subjectNr = count
        self.settings = None
        self.stims = None
        self.stimsClassified = 0
        self.actualLabels = None
        self.predictedLabels = None
        self.meanPredictTime = 0.0
        self.acc = 0.0


dataDir = '/home/tonnius/Git/bci_ssvep_openvibe_ros/OpenVibe/results'  # Directory to search for results
dataDirFileNames = sorted(os.listdir(dataDir))
data = pickle.load(open(os.path.join(dataDir, dataDirFileNames[0]), 'rb'))
NR_OF_SUBJECTS = data['settings']['nrOfSubjects']
NR_OF_STATES = 4

maxResList = [MaxRes() for count in xrange(NR_OF_SUBJECTS)]  # Holds results

for fileN in dataDirFileNames:
    if fileN == 'Readme.txt':
        continue

    data = pickle.load(open(os.path.join(dataDir, fileN), 'rb'))
    experimentSettings = data['settings']
    actualLabels = data['actual']
    predictedLabels = data['predicted']
    stimsData = data['stims']

    meanDetectTime = (data['detectTime']*0.75) + 0.25*((stimsData['stimsNrActual'] *
                    (7 - data['detectTime'])) / (stimsData['totalNrClassified'] - stimsData['stimsNrClassified']))

    if actualLabels and predictedLabels:
        cmSklearn = confusion_matrix(actualLabels, predictedLabels)
        cmSklearn = cmSklearn.astype('float') / cmSklearn.sum(axis=1)[:, np.newaxis]
        perClassAccs = [False, False, False, False]

        # Minimum accuracy condition
        acc = 0.0
        for i in range(4):
            acc += cmSklearn[i][i]
            if cmSklearn[i][i] >= 0.7:
                perClassAccs[i] = True
        perClassAccCond = all(item is True for item in perClassAccs)
        acc /= 4

        # Calc ITR
        if acc == 1.0:
            itr = (math.log(NR_OF_STATES, 2) + acc*math.log(acc, 2)) * (60.0 / meanDetectTime)
        elif acc == 0.0:
            itr = 0
        else:
            itr = (math.log(NR_OF_STATES, 2) + acc*math.log(acc, 2) + (1.0 - acc) *
                   math.log((1.0 - acc)/(NR_OF_STATES - 1.0),2)) * (60.0 / meanDetectTime)
        subjectNr = int(experimentSettings['currentSubjNr'])

        stimsClassified = stimsData['stimsNrClassified']
        stimsTotal = stimsData['stimsNrActual']

        # Save results if better
        if itr > maxResList[subjectNr].maxItr and meanDetectTime < 7 and acc > 0.7:
            if stimsClassified >= maxResList[subjectNr].stimsClassified:
                maxResList[subjectNr].maxItr = itr
                maxResList[subjectNr].maxItrFileN = fileN
                maxResList[subjectNr].maxCmSklearn = cmSklearn
                maxResList[subjectNr].settings = experimentSettings
                maxResList[subjectNr].actualLabels = actualLabels
                maxResList[subjectNr].predictedLabels = predictedLabels
                maxResList[subjectNr].stims = stimsData
                maxResList[subjectNr].stimsClassified = stimsClassified
                maxResList[subjectNr].meanPredictTime = meanDetectTime
                maxResList[subjectNr].acc = acc

# Print results
for res in maxResList:
    print "Subject {0} had max ITR {1} with settings {2}".format(res.subjectNr, res.maxItr, res.settings)
    print "Confusion matrix: actual are rows, predicted are columns"
    print res.maxCmSklearn
    print "stims data "+str(res.stims)
    print "mean detect time "+str(res.meanPredictTime)
    print "accuracy was " + str(res.acc)
    print classification_report(res.actualLabels, res.predictedLabels, digits=4)