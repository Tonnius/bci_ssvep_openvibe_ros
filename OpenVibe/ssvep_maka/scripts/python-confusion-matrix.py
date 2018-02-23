import numpy as np
#from sklearn.metrics import confusion_matrix
#from pandas_ml import ConfusionMatrix

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
		for chunkIndex in range( len(self.input[0]) ):
			chunk = self.input[0].pop()
			if(type(chunk) == OVStimulationSet):
				for stimIdx in range(len(chunk)):
					stim=chunk.pop();
					#print 'Received stim on input0', stim.identifier, 'stamped at', stim.date, 's'
					self.actualLabels.append(stim.identifier)

		for chunkIndex1 in range( len(self.input[1]) ):
			chunk1 = self.input[1].pop()
			if(type(chunk1) == OVStimulationSet):
				for stimIdx1 in range(len(chunk1)):
					stim1=chunk1.pop();
					#if stim is not 0:
						#if stim1.date > stim.date:
					#print 'Received stim on input1', stim1.identifier, 'stamped at', stim1.date, 's'
					self.predictedLabels.append(stim1.identifier)


			#else:
			#	print 'Received chunk of type ', type(chunk), " looking for StimulationSet"
		return
		
	def uninitialize(self):
		with open('subj1', 'wb') as f:
			np.savez(f, self.actualLabels, self.predictedLabels)
		print 'File saved to '
		#print 'actual labels array size: ', len(self.actualLabels), ' predictedLabels array size: ', len(self.predictedLabels)
		#confusion_matrix = ConfusionMatrix(self.actualLabels, self.predictedLabels)
		#print("Confusion matrix:\n%s" % confusion_matrix)
		#confusion_matrix.print_stats()

		

box = MyOVBox()
