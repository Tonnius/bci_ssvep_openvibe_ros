#!/bin/bash

OPENVIBE_ROOT_PATH="$HOME/Git/openvibe/dist/extras-Release"
MY_SCRIPT_PATH="$HOME/Git/magister_BCI/OpenVibe/ssvep_maka"
#INPUT_TRAINING_FILE_LOC="$MY_SCRIPT_PATH/signals/ssvep-record-[2018.02.13-15.22.10]-martin-8-5elect.ov"
#INPUT_TESTING_FILE_LOC="$MY_SCRIPT_PATH/signals/ssvep-record-[2018.02.13-15.31.38]-martin-test-no-zero-8-5-elect.ov"

declare -a trainingArr=("$MY_SCRIPT_PATH/signals/ssvep-record-[2018.02.13-15.22.10]-martin-8-5elect.ov" 
                "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.02.15-14.20.15]-teet.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.01.10-15.32.49]-tonis.ov"                
                )

declare -a testingArr=("$MY_SCRIPT_PATH/signals/ssvep-record-[2018.02.13-15.31.38]-martin-test-no-zero-8-5-elect.ov" 
                "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.02.15-14.32.32]-teet-test-no-zero.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.01.17-16.49.48]-tonis-test.ov"
                )

PROC_EPOCH_DUR=0.1
PROC_EPOCH_INTER=0.1
FREQ_TOLERANCE=0.1
CHANNELS="1:4"


sudo "$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/ssvep-configuration-auto.xml" --invisible \
--define epDur $PROC_EPOCH_DUR \
--define epInt $PROC_EPOCH_INTER \
--define freqTol $FREQ_TOLERANCE \
--define ch $CHANNELS
wait

arraylength=${#trainingArr[@]}

for ((i=0; i<${arraylength}; i++));
do

	sudo "$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/CSP-training-auto.xml" --define inFile ${trainingArr[$i]} --invisible 
	wait
	sudo "$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/classifier-training-auto.xml" --define inFile ${trainingArr[$i]} --invisible
	wait
	sudo "$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/perf-measure-auto.mxs" --define inFile ${testingArr[$i]} --invisible \
	--define epDur $PROC_EPOCH_DUR \
    --define epInt $PROC_EPOCH_INTER \
    --define freqTol $FREQ_TOLERANCE \
	--define ch $CHANNELS
	wait

	echo $i " / " ${arraylength} " : " ${trainingArr[$i]}
done