import pickle
import os
from pandas_ml import ConfusionMatrix
import numpy as np
import math
from sklearn.metrics import confusion_matrix
from sklearn.metrics import f1_score
from sklearn.metrics import classification_report
class maxRes():
    def __init__(self, count):
        self.maxCmPanda = None
        self.maxCmSklearn = None
        self.maxItr = 0
        self.maxItrFileN = ''
        self.subjectNr = count
        self.settings = None
        self.stimsClassified = 0
        self.stimsTotal = 0
        self.actualLabels = None
        self.predictedLabels = None
        self.meanPredictTime = 0.0
        self.acc = 0.0

dataDir = '/home/tonnius/Git/magister_BCI/OpenVibe/ssvep_maka/data/'
dataDirFileNames = sorted(os.listdir(dataDir))
#fileNames = dataDirFileNames[1:]
data = pickle.load(open(os.path.join(dataDir, dataDirFileNames[0]), 'rb'))
NR_OF_SUBJECTS = data['settings']['nrOfSubjects']
NR_OF_STATES = 4

#for count in xrange(NR_OF_SUBJECTS):
maxResList = [maxRes(count) for count in xrange(NR_OF_SUBJECTS)]

#maxItr = 0
#maxItrFileN = ''
#maxValues = []
for fileN in dataDirFileNames:
    data = pickle.load(open(os.path.join(dataDir, fileN), 'rb'))
    experimentSettings = data['settings']
    actualLabels = data['actual']
    predictedLabels = data['predicted']
    meanDetectTime = data['detectTime']
    stimsNrs = data['stims']
    if actualLabels and predictedLabels:
        cmPanda = ConfusionMatrix(actualLabels, predictedLabels)
        cmSklearn = confusion_matrix(actualLabels, predictedLabels)
        # print("Confusion matrix:\n%s" % confusion_matrix)
        result = cmPanda.stats()
        acc = result['overall']['Accuracy']
        if acc == 1.0:
            itr = (math.log(NR_OF_STATES, 2) + acc*math.log(acc, 2)) * (60.0 / meanDetectTime)
        elif acc == 0.0:
            itr = 0
        else:
            itr = (math.log(NR_OF_STATES, 2) + acc*math.log(acc, 2) + (1.0-acc)*math.log((1.0 - acc)/(NR_OF_STATES - 1.0),2)) * (60.0 / meanDetectTime)
        subjectNr = int(experimentSettings['currentSubjNr'])

        stimsClassified = stimsNrs['stimsNrClassified']
        stimsTotal = stimsNrs['stimsNrActual']

        if itr > maxResList[subjectNr].maxItr and acc > 0.7 and meanDetectTime < 5:
            if stimsClassified >= maxResList[subjectNr].stimsClassified:
                maxResList[subjectNr].maxItr = itr
                maxResList[subjectNr].maxItrFileN = fileN
                maxResList[subjectNr].maxCmPanda = cmPanda
                maxResList[subjectNr].maxCmSklearn = cmSklearn
                maxResList[subjectNr].settings = experimentSettings
                maxResList[subjectNr].actualLabels = actualLabels
                maxResList[subjectNr].predictedLabels = predictedLabels
                maxResList[subjectNr].stimsClassified = stimsClassified
                maxResList[subjectNr].stimsTotal = stimsTotal
                maxResList[subjectNr].meanPredictTime = meanDetectTime
                maxResList[subjectNr].acc = acc

for res in maxResList:
    print "Subject {0} had max ITR {1} with settings {2}".format(res.subjectNr, res.maxItr, res.settings)
    #res.maxCmPanda.print_stats()

    res.maxCmSklearn = res.maxCmSklearn.astype('float') / res.maxCmSklearn.sum(axis=1)[:, np.newaxis]
    print(res.maxCmSklearn)
    print "stims classified "+str(res.stimsClassified)+" / all "+str(res.stimsTotal)
    print "mean detect time "+str(res.meanPredictTime)
    print "accuracy was " + str(res.acc)
    print(classification_report(res.actualLabels, res.predictedLabels))
