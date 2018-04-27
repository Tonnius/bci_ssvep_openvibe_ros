#!/bin/bash
trap "exit" INT

OPENVIBE_ROOT_PATH="$HOME/Git/openvibe/dist/extras-Release"
MY_SCRIPT_PATH="$HOME/Git/magister_BCI/OpenVibe/ssvep_maka"

rm -rf "$MY_SCRIPT_PATH/data/"*

declare -a trainingArr=(
				"$MY_SCRIPT_PATH/signals/ssvep-record-[2018.02.13-15.22.10]-martin-8-5elect.ov" 
                "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.02.15-14.20.15]-teet.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.01.10-15.32.49]-tonis.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.02.16-12.29.19]-anti.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.03.08-15.10.29]-klavs.ov" 
                "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.03.09-15.52.56]-frida.ov"
                "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.03.27-13.35.05]-annika-75-857-10-12Hz.ov"
               "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.03.26-11.45.32]-t6nis-75-857-10-12Hz.ov"                  
               )

declare -a testingArr=(
				"$MY_SCRIPT_PATH/signals/ssvep-record-[2018.02.13-15.31.38]-martin-test-no-zero-8-5-elect-2.csv" 
               "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.02.15-14.32.32]-teet-test-no-zero-2.csv"
               "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.01.10-16.10.07]-tonis-test-2.csv"
               "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.02.16-12.41.41]-anti-test-2.csv"
               "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.03.08-15.24.56]-klavs-test-2.csv"
               "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.03.09-16.05.30]-frida-test-2.csv"
               "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.03.27-13.46.46]-annika-75-857-10-12Hz-test-2.csv"
                "$MY_SCRIPT_PATH/signals/ssvep-record-[2018.03.26-11.57.22]-t6nis-75-857-10-12Hz-test-2.csv"
                )
parameterIdent="epDur"
declare -a parametersEpDur=(1.0 0.9 0.8 0.7 0.6 0.5)
#declare -a parametersEpInt=(0.01 0.025 0.05 0.075 0.1 0.2)
declare -a parametersFreqTol=(0.4 0.3 0.25 0.5)
#declare -a parametersThresh=("0.5:0.3" "0.5:0.2" "0.4:0.35" "0.6:0.2" "0.7:0.1")
declare -a parametersThresh=("0.5:0.2" "0.5:0.25" "0.5:0.15" "0.55:0.2" "0.4:0.35" "0.4:0.4" "0.4:0.3")

if [ "$parameterIdent" = "epDur" ]; then
    declare -a parameters=( ${parametersEpDur[@]} )
fi
if [ "$parameterIdent" = "freqTol" ]; then
    declare -a parameters=( ${parametersFreqTol[@]} )
fi
if [ "$parameterIdent" = "thresh" ]; then
    declare -a parameters=( ${parametersThresh[@]} )
fi

PROC_EPOCH_DUR=0.5
PROC_EPOCH_INTER=0.5

FREQ_TOLERANCE=$1

TRESH=$2

paramArraylength=${#parameters[@]}


arraylength=${#trainingArr[@]}
#echo $1

k=0
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

		if (( $parameterIdent == "epDur" || $parameterIdent == "freqTol")) || (( $parameterIdent == "thresh" && $k == 0 )); then
		#if [ "$parameterIdent" = "epDur" ] || [ "$parameterIdent" = "freqTol" ]; then

			"$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/ssvep-configuration-auto.xml" --invisible \
				--define epDur $PROC_EPOCH_DUR \
				--define epInt $PROC_EPOCH_INTER \
				--define freqTol $FREQ_TOLERANCE \
				--define ch $CHANNELS \
				--define simFreq1 $SIM_FREQ1 \
				--define simFreq2 $SIM_FREQ2 \
				--define simFreq3 $SIM_FREQ3 \
				--define simFreq4 $SIM_FREQ4 
			wait
			"$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/CSP-training-harm-auto.xml" --define inFile ${trainingArr[$i]} --invisible 
			wait
			"$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/classifier-training-harm-auto.xml" --define inFile ${trainingArr[$i]} --invisible
			wait
		fi 
		"$OPENVIBE_ROOT_PATH/openvibe-designer.sh" --play-fast "$MY_SCRIPT_PATH/perf-measure-harm-auto.mxs" --define inFile ${testingArr[$i]} --invisible \
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