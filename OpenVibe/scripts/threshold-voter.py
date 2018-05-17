import numpy as np
import os

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
        self.predNothingEnabled = True
        self.allProbsMean = [0.0, 0.0, 0.0, 0.0]
        self.underThreshValues = [0, 0, 0, 0]

    def initialize(self):
        threshString = self.setting['Thresholds']
        threshs = threshString.split(':')
        if threshs:
            self.classThresh = float(threshs[0])
            self.maxProbDiffThresh = float(threshs[1])
        else:
            self.classThresh = 0.5
            self.maxProbDiffThresh = 0.25
        return

    def getProbValue(self, inputNr, classNr):
        for chunkIndexMatrix in range(len(self.input[inputNr])):
            probMatrix = self.input[inputNr].pop() #probMatrix[0] is stimulated, probMatrix[1] is non-stimulated
            self.classProbs[classNr].append([probMatrix[0], self.getCurrentTime()])
            if probMatrix[0] > self.classThresh:
                return True

        return False

    def process(self):

        probsAll = [0.0, 0.0, 0.0, 0.0]

        classHit = [self.getProbValue(inputNr=3, classNr=0),
                    self.getProbValue(inputNr=4, classNr=1),
                    self.getProbValue(inputNr=5, classNr=2),
                    self.getProbValue(inputNr=6, classNr=3)]


        if True in classHit:
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

            if (maxPosDif >= self.maxProbDiffThresh):
                print "predicted class was " + str(maxpos + 1)
                stimSet = OVStimulationSet(self.getCurrentTime(), self.getCurrentTime() + 1. / self.getClock())
                # the date of the stimulation is simply the current openvibe time when calling the box process
                stimSet.append(OVStimulation(33025 + maxpos, self.getCurrentTime(), 0.))
                self.output[0].append(stimSet)
                #self.output[0].append(OVStimulation(33025 + maxpos, self.getCurrentTime(), 0.0))

        return

    def uninitialize(self):
        return

box = MyOVBox()
