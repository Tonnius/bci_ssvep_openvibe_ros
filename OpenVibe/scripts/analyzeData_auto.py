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
import random
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

randomIterations = 6

for j in range(randomIterations):
    thresh1 = random.uniform(0.3, 0.7)
    # if thresh1 < 0.25:
    #     thresh1 = 0.25
    # elif thresh1 > 0.75:
    #     thresh1 = 0.75
    thresh2 = random.uniform(0.1, 0.5)
    # if thresh2 < 0.05:
    #     thresh1 = 0.05
    # elif thresh2 > 0.5:
    #     thresh2 = 0.5
    thresh = str(thresh1)+":"+str(thresh2)
    epDur = 0.5 + j*0.1
    print epDur
    freqTol = str(random.uniform(0.1, 0.5))
    scriptDir = '/home/tonnius/Git/magister_BCI/OpenVibe/ssvep_maka/harmAll_auto.sh'
    if False:
        p = subprocess.call([scriptDir, freqTol, thresh])
        while p != 0:
            print "waiting"
            time.sleep(5)

    dataDir = '/home/tonnius/Dropbox/Magistritoo/katseTulemused/data_epDur_opt_svm'
    #dataDir = '/home/tonnius/Git/magister_BCI/OpenVibe/ssvep_maka/data'

    dataDirFileNames = sorted(os.listdir(dataDir), key=MyFn)

    data = pickle.load(open(os.path.join(dataDir, dataDirFileNames[0]), 'rb'))
    NR_OF_SUBJECTS = data['settings']['nrOfSubjects']
    NR_OF_STATES = 4
    nrOfFiles = len(dataDirFileNames)
    NR_OF_CONFS = int(nrOfFiles / NR_OF_SUBJECTS)
    overallITR = np.zeros((NR_OF_SUBJECTS, NR_OF_CONFS))
    overAllSettings = np.empty((NR_OF_SUBJECTS, NR_OF_CONFS), dtype=object)
    overallAcc= np.zeros((NR_OF_SUBJECTS, NR_OF_CONFS))
    overallMDT = np.zeros((NR_OF_SUBJECTS, NR_OF_CONFS))
    maxResList = [maxRes(count) for count in xrange(NR_OF_SUBJECTS)]


    for idx,fileN in enumerate(dataDirFileNames):
        data = pickle.load(open(os.path.join(dataDir, fileN), 'rb'))
        experimentSettings = data['settings']
        actualLabels = data['actual']
        predictedLabels = data['predicted']
        stimsData = data['stims']
        meanDetectTime = (data['detectTime']*0.75)+ \
                0.25*((stimsData['stimsNrActual']*(7-data['detectTime']))/(stimsData['totalNrClassified']-stimsData['stimsNrClassified']))

        if actualLabels and predictedLabels and experimentSettings['epDur'] == str(epDur):
                #and  experimentSettings['freqTol'] == '0.25' \
                 #   and experimentSettings['classTresh'] == 0.5 and experimentSettings['maxProbDiffThresh'] == 0.25:
            #cmPanda = ConfusionMatrix(actualLabels, predictedLabels)
            cmSklearn = confusion_matrix(actualLabels, predictedLabels)
            cmSklearn = cmSklearn.astype('float') / cmSklearn.sum(axis=1)[:, np.newaxis]
            perClassAccs = [False, False, False, False]
            acc = 0.0
            subjectNr = int(experimentSettings['currentSubjNr'])
            #if subjectNr == 0:
            #    print "here!"
            for i in range(4):
                acc += cmSklearn[i][i]
                if cmSklearn[i][i] >= 0.0:
                    perClassAccs[i] = True

            perClassAccCond = all(item is True for item in perClassAccs)
            acc /= 4.0
            #if acc > 0.7:
            #    perClassAccCond = True
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
            if perClassAccCond and MDTCond:
                overallAcc[subjectNr][index]= acc
                overallMDT[subjectNr][index]=meanDetectTime
                overAllSettings[subjectNr][index]=experimentSettings
                overallITR[subjectNr][index] = itr


    ITRsumBest = 0

    ITRsum = overallITR.sum(axis=0)
    bestSettingPos = np.argmax(ITRsum)
    minItr = np.min(overallITR[np.nonzero(overallITR)])
    maxItr = np.max(overallITR[np.nonzero(overallITR)])



    res = np.take(overAllSettings, bestSettingPos,axis=1)
    avgAcc = sum(np.take(overallAcc, bestSettingPos, axis=1))/NR_OF_SUBJECTS
    avgMDT = sum(np.take(overallMDT, bestSettingPos, axis=1))/NR_OF_SUBJECTS
    avgITR = sum(ITRsum)/NR_OF_SUBJECTS
    if (avgITR > globalITRAvg) or True:
        outStr = 'Best settings '+str(res)
        print " maxITR: "+str(maxItr)+" minItr: "+str(minItr)+" avg ITR: "+str(max(ITRsum)/NR_OF_SUBJECTS)
        print "minACC: "+str(np.min(overallAcc[np.nonzero(overallAcc)]))+" maxAcc "+str(np.max(overallAcc[np.nonzero(overallAcc)]))+" avgAcc: "+str(avgAcc)
        print "minMDT: " + str(np.min(overallMDT[np.nonzero(overallMDT)])) + " maxMDT " + str(np.max(overallMDT[np.nonzero(overallMDT)]))+" avgMDT: "+str(avgMDT)
        #with open('bestRes.txt', 'a') as file:
         #   file.write(outStr)

        globalITRAvg = avgITR
        print outStr

    print "END OF ITERATION "+str(j)