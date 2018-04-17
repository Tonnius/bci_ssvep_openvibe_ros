import pickle
import os
from pandas_ml import ConfusionMatrix
import numpy as np
import math
from sklearn.metrics import confusion_matrix
from sklearn.metrics import f1_score
from sklearn.metrics import classification_report
import subprocess
import time
import math

globalITRAvg = 0.0

def MyFn(s):
    res = s[2:].split('subject')
    return res[1]+res[0]
class maxRes():
    def __init__(self, count):
        self.maxCmPanda = None
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
        self.thresh = None

thresh = "0.5:0.25"
epDur = 0.5
freqTol = '0.25'
scriptDir = '/home/tonnius/Git/magister_BCI/OpenVibe/ssvep_maka/harmAll_auto.sh'
if True:
    p = subprocess.call([scriptDir, freqTol, thresh])
    #p.wait()
    while p != 0:
        print "waiting"
        time.sleep(5)

#dataDir = '/home/tonnius/Dropbox/Magistritoo/katseTulemused/data-origLda'
dataDir = '/home/tonnius/Git/magister_BCI/OpenVibe/ssvep_maka/data'

dataDirFileNames = sorted(os.listdir(dataDir), key=MyFn)

data = pickle.load(open(os.path.join(dataDir, dataDirFileNames[0]), 'rb'))
NR_OF_SUBJECTS = data['settings']['nrOfSubjects']
NR_OF_STATES = 4
nrOfFiles = len(dataDirFileNames)
NR_OF_CONFS = int(nrOfFiles / NR_OF_SUBJECTS)
overallITR = np.zeros((NR_OF_SUBJECTS, NR_OF_CONFS))
overAllSettings = np.empty((NR_OF_SUBJECTS, NR_OF_CONFS), dtype=object)

maxResList = [maxRes(count) for count in xrange(NR_OF_SUBJECTS)]


for idx,fileN in enumerate(dataDirFileNames):
    data = pickle.load(open(os.path.join(dataDir, fileN), 'rb'))
    experimentSettings = data['settings']
    actualLabels = data['actual']
    predictedLabels = data['predicted']
    stimsData = data['stims']
    meanDetectTime = (data['detectTime']*0.75)+ \
            0.25*((stimsData['stimsNrActual']*(7-data['detectTime']))/(stimsData['totalNrClassified']-stimsData['stimsNrClassified']))

    if actualLabels and predictedLabels:
        #cmPanda = ConfusionMatrix(actualLabels, predictedLabels)
        cmSklearn = confusion_matrix(actualLabels, predictedLabels)
        cmSklearn = cmSklearn.astype('float') / cmSklearn.sum(axis=1)[:, np.newaxis]
        perClassAccs = [False, False, False, False]
        acc = 0.0
        subjectNr = int(experimentSettings['currentSubjNr'])

        for i in range(4):
            acc += cmSklearn[i][i]
            if cmSklearn[i][i] >= 0.4:
                perClassAccs[i] = True

        perClassAccCond = all(item is True for item in perClassAccs)
        acc /= 4.0
        #print(cmSklearn)
        # print("Confusion matrix:\n%s" % confusion_matrix)
        #result = cmPanda.stats()
        #acc = result['overall']['Accuracy']
        if acc == 1.0:
            itr = (math.log(NR_OF_STATES, 2) + acc*math.log(acc, 2)) * (60.0 / meanDetectTime)
        elif acc == 0.0:
            itr = 0
        else:
            itr = (math.log(NR_OF_STATES, 2) + acc*math.log(acc, 2) + (1.0-acc)*math.log((1.0 - acc)/(NR_OF_STATES - 1.0),2)) * (60.0 / meanDetectTime)

        MDTCond = meanDetectTime < 7
        stimsClassified = stimsData['stimsNrClassified']
        stimsTotal = stimsData['stimsNrActual']
        index = int(math.floor(idx%NR_OF_CONFS))
        if perClassAccCond and MDTCond and itr > 20.0:

            overAllSettings[subjectNr][index]=experimentSettings
            overallITR[subjectNr][index] = itr

        # if itr > maxResList[subjectNr].maxItr and perClassAccCond and MDTCond: #and experimentSettings['epDur'] == '0.75':
        #     if stimsClassified >= maxResList[subjectNr].stimsClassified:
        #         maxResList[subjectNr].maxItr = itr
        #         maxResList[subjectNr].maxItrFileN = fileN
        #         maxResList[subjectNr].maxCmSklearn = cmSklearn
        #         maxResList[subjectNr].settings = experimentSettings
        #         maxResList[subjectNr].actualLabels = actualLabels
        #         maxResList[subjectNr].predictedLabels = predictedLabels
        #         maxResList[subjectNr].stims = stimsData
        #         maxResList[subjectNr].stimsClassified = stimsClassified
        #         maxResList[subjectNr].meanPredictTime = meanDetectTime
        #         maxResList[subjectNr].acc = acc


# for res in maxResList:
#     if res.maxCmSklearn is not None:
#         print "Subject {0} had max ITR {1} with settings {2}".format(res.subjectNr, res.maxItr, res.settings)
#         #res.maxCmPanda.print_stats()
#
#         res.maxCmSklearn = res.maxCmSklearn.astype('float') / res.maxCmSklearn.sum(axis=1)[:, np.newaxis]
#         print(res.maxCmSklearn)
#         print "stims data "+str(res.stims)
#         print "mean detect time "+str(res.meanPredictTime)
#         print "accuracy was " + str(res.acc)
#         print(classification_report(res.actualLabels, res.predictedLabels, digits=4))

ITRsumBest = 0

ITRsum = overallITR.sum(axis=0)
bestSettingPos = np.argmax(ITRsum)
res = np.take(overAllSettings, bestSettingPos,axis=1)
avgITR = sum(ITRsum)/NR_OF_SUBJECTS
outStr = 'Best global settings so far:'+str(res)+"at avg ITR "+str(max(ITRsum)/NR_OF_SUBJECTS)+'\n'
if (avgITR > globalITRAvg):
    with open('bestRes.txt', 'a') as file:
        file.write(outStr)
    globalITRAvg = avgITR
print outStr