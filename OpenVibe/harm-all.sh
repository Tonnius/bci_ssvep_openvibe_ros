#!/bin/bash

# Script for running OpenVibe programmatically in a loop

trap "exit" INT

OPENVIBE_ROOT_PATH="$HOME/Git/openvibe/dist/extras-Release"
MY_SCRIPT_PATH="$HOME/Git/bci_ssvep_openvibe_ros/OpenVibe"

#rm -rf "$MY_SCRIPT_PATH/data/"*

declare -a trainingArr=(
				"$MY_SCRIPT_PATH/signals/ssvep-record-subject1.ov" 
                "$MY_SCRIPT_PATH/signals/ssvep-record-subject2.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-subject3.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-subject4.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-subject5.ov" 
                "$MY_SCRIPT_PATH/signals/ssvep-record-subject6.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-subject7.ov" 
                "$MY_SCRIPT_PATH/signals/ssvep-record-subject8.ov"                
               )

declare -a testingArr=(
				"$MY_SCRIPT_PATH/signals/ssvep-record-subject1-test.ov" 
                "$MY_SCRIPT_PATH/signals/ssvep-record-subject2-test.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-subject3-test.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-subject4-test.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-subject5-test.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-subject6-test.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-subject7-test.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-subject8-test.ov"
                )

parameterIdent2="freqTol"
parameterIdent="epDur"

declare -a parametersEpDur=(0.5)
#declare -a parametersEpInt=(0.5)
declare -a parametersFreqTol=(0.25)
declare -a parametersThresh=("0.5:0.1")


if [ "$parameterIdent" = "epDur" ]; then
    declare -a parameters=( ${parametersEpDur[@]} )
fi
#if [ "$parameterIdent" = "epInt" ]; then
#    declare -a parameters=( ${parametersEpInt[@]} )
#fi
if [ "$parameterIdent" = "freqTol" ]; then
    declare -a parameters=( ${parametersFreqTol[@]} )
fi
if [ "$parameterIdent" = "thresh" ]; then
    declare -a parameters=( ${parametersThresh[@]} )
fi


if [ "$parameterIdent2" = "freqTol" ]; then
    declare -a parameters2=( ${parametersFreqTol[@]} )
fi
if [ "$parameterIdent2" = "thresh" ]; then
    declare -a parameters2=( ${parametersThresh[@]} )
fi
PROC_EPOCH_DUR=0.5
PROC_EPOCH_INTER=0.5
FREQ_TOLERANCE=0.3


TRESH="0.46:0.22"
paramArraylength=${#parameters[@]}
paramArraylength2=${#parameters2[@]}
arraylength=${#trainingArr[@]}

for ((i=0; i<${arraylength}; i++));
do
	SIM_FREQ1="20"
	SIM_FREQ2="15"
	SIM_FREQ3="12"
	SIM_FREQ4="10"
	CHANNELS="1:5"
	if [ "$i" -ge "4" ]; then
    	SIM_FREQ1="30"
		SIM_FREQ2="20"
		SIM_FREQ3="12"
		SIM_FREQ4="7.5"

	fi 
	if [ "$i" -ge "6" ]; then
    	SIM_FREQ1="12"
		SIM_FREQ2="10"
		SIM_FREQ3="8.57142857143"
		SIM_FREQ4="7.5"

	fi

	for ((k=0; k<${paramArraylength}; k++));
	do
		if [ "$parameterIdent" = "epDur" ]; then
	        PROC_EPOCH_DUR=${parameters[$k]}
	        PROC_EPOCH_INTER=${parameters[$k]}
	    fi
	    if [ "$parameterIdent" = "freqTol" ]; then
	        FREQ_TOLERANCE=${parameters[$k]}
	    fi
	    if [ "$parameterIdent" = "thresh" ]; then
	        TRESH=${parameters[$k]}
	    fi

	    for ((j=0; j<${paramArraylength2}; j++));
		do
			if [ "$parameterIdent2" = "freqTol" ]; then
        		FREQ_TOLERANCE=${parameters2[$j]}
    		fi
    		if [ "$parameterIdent2" = "thresh" ]; then
        		TRESH=${parameters2[$j]}
    		fi
			if (( $parameterIdent == "epDur" )) || (( $parameterIdent2 == "thresh" && $j == 0 )); then
				sudo "$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/ssvep-configuration.xml" --invisible \
					--define epDur $PROC_EPOCH_DUR \
					--define epInt $PROC_EPOCH_INTER \
					--define freqTol $FREQ_TOLERANCE \
					--define ch $CHANNELS \
					--define simFreq1 $SIM_FREQ1 \
					--define simFreq2 $SIM_FREQ2 \
					--define simFreq3 $SIM_FREQ3 \
					--define simFreq4 $SIM_FREQ4 
				wait
				sudo "$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/CSP-training-harm.xml" --define inFile ${trainingArr[$i]} --invisible 
				wait
				sudo "$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/classifier-training-harm.xml" --define inFile ${trainingArr[$i]} --invisible
				wait
			fi 
			sudo "$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/perf-measure-harm.mxs" --define inFile ${testingArr[$i]} --invisible \
				--define epDur $PROC_EPOCH_DUR \
			    --define epInt $PROC_EPOCH_INTER \
			    --define freqTol $FREQ_TOLERANCE \
				--define ch $CHANNELS \
				--define nrOfSubjects ${arraylength} \
				--define simFreq1 $SIM_FREQ1 \
				--define simFreq2 $SIM_FREQ2 \
				--define simFreq3 $SIM_FREQ3 \
				--define simFreq4 $SIM_FREQ4 \
				--define currentSubjNr $i \
				--define thresh $TRESH
			wait
		done
	done
done