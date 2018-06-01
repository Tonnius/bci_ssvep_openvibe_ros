# VEP-based BCI using OpenVibe and ROS
## Introduction
This repository was initially produced in the process of an MSc thesis at the University of Tartu. The main purpose of this repository is to give an overview of the developed OpenVibe and ROS packages for setting up the brain-computer interface (BCI) for robot control.

## Setup
The whole system was set up on Ubuntu 16.04 because of ROS, but the BCI part on OpenVibe can be tested on any other OS. 
### OpenVibe setup
* Linux - download the OpenVibe [source code](http://openvibe.inria.fr/downloads/) and see [build instructions](http://openvibe.inria.fr/build-instructions/) for details.
* Windows - download [Windows installer](http://openvibe.inria.fr/downloads/).

The OpenVibe folder in this repository contains all the neccessary files for using the system. The following is a list explaining the most important files and the order they should be used:

1. ssvep-configuration.xml - for setting up system parameters
2. training-acquisition.xml - for training data acquisition
3. CSP-training-harm.xml - for training the CSP spatial filters
4. classifier-training-harm.xml - for training the classifiers
5. online-4-stim.mxs - for using the system online with 4 stimuli

Also there is *acquisition-test.xml* for looking at raw EEG data and *perf-measure-harm.mxs* for performance measurements.

NOTE: by default, OpenVibe is set up to display 3 stimuli. To configure the number of stimuli, the files *openvibe-ssvep-demo.conf* and *trainer.conf* in the OpenVibe installation directory must be edited. Examples for 4 stimuli are in the bci_ssvep_openvibe_ros/OpenVibe/ssvep-demo folder of this repository.

### ROS setup

TODO

