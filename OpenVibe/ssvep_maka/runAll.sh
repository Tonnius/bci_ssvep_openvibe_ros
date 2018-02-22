#!/bin/bash

OPENVIBE_ROOT_PATH="$HOME/Git/openvibe/dist/extras-Release"
MY_SCRIPT_PATH="$HOME/Git/openvibe/dist/extras-Release/share/openvibe/scenarios/bci-examples/ssvep_maka"
INPUT_TRAINING_FILE_LOC="$MY_SCRIPT_PATH/signals/ssvep-record-[2018.02.09-12.25.10].ov"

PROC_EPOCH_DUR=0.25
PROC_EPOCH_INTER=0.1
FREQ_TOLERANCE=0.25
CHANNELS="1:4"
sudo "$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/ssvep-bci-1-ssvep-configuration.xml" --invisible \
--define epDur $PROC_EPOCH_DUR \
--define epInt $PROC_EPOCH_INTER \
--define freqTol $FREQ_TOLERANCE \
--define ch $CHANNELS
wait
#sudo "$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/ssvep-bci-3-CSP-training.xml" --define inFile "$INPUT_TRAINING_FILE_LOC" --invisible 
#wait
#sudo "$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/ssvep-bci-4-classifier-training.xml" --invisible
#wait
#sudo "$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/ssvep-perf-measure.mxs" --invisible

