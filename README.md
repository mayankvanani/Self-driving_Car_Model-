# Self-driving_Car_Model-PROJECT

## Overview
Python program was built ot make a car-model a self-driving technology. Uses extensive Image processing(opencv) to detected lane edges. It uses **ssd_mobilenet_v1_coco_11_06_2017** model to classify the obstacle on its path and avoid collision by steering clear through it. 

**_NOTE_**: This code is developed for campus road (Nirma University). So it needs to adapted according to the road condition.

## Pre-requisites
* Cuda 9.0
* Cudnn 7.0.5
* Tensorflow 1.5 or greater
* Download: Tensorflow-models repository [Github](https://github.com/tensorflow/models)
* Arduino IDE
* pyfirmata.whl

## User Manual
* copy the listed below python files to _" models/research/object_detection/ "_
  - **self-driving_with_arduino_interface.py**
  - **self_driving_without_arduino_interface.py**

* Open Arduino IDE and load **standard-firmata** and dump it in arduino. 

**_NOTE_**: 
* _self-driving_with_arduino_interface.py_ will require an arduino connected to laptop to run successfullly. Without arduino           connected, it will not run and throw an error.
* _self_driving_without_arduino_interface.py_ will run without any hardware connected. It will pop up camera point of view, classify and generate steering output.

## Hardware
* Camera_: any 2D camera
* IR sensors: Short Range(30cm to 150cm) and Long Range(150cm to 550cm)
* Arduino Mega (for parsing the actuating signals to rear motors and steering motor)
* GPU (used the laptop with inbuilt Nvidia GPU-940MX with computing capability-5.0) : required for processing

## Description
### Camera Output Screen
It contains various lines markers in blue and red. Red markers identify the left and right edges of the campus road. Blue marker is the steering advice generated based upon the red marker positioning. A blue box in the middle of the screen is the proportional dead-band that is made to eradicate the minute steering correction that occurs constantly. Also flase Steering Advice generated due to bumps or some internal error is also taken care of by optimising the code and removing the occurance of such situation.

### IR sensor Fusion
Adaptive Cruise Control is achieved through IR sensors. The front zone was divided into various segments and speed in each zone was defined by mathematical equation obtained by calibrating the IR sensor value and generateing PWM accordingly to manipulate speed. The value is of both the sensors is checked in each iteration and speed is updated continuously.The update of speed is made gradual through code optimisation.

### Obstacle Detection and Avoidance.
SSD_mobilenet_coco (Commom Object in COntest) is the model used ot detect and classify object and based on the boxes drawn by the classifier, the relative distance is calculated according to the frame. Based upon its size, the algorithm decides wheather the car can steer pass it or it has to stop. Reason behind choosing this model is that its compulationaly less complex and hence frames obtained is 15fps.

### ALGORITHM
1. Check the short range IR value
2. Check Long Range IR value
3. Genreate PWM to modulate speed based upon the IR values
4. Steering Advice based on markers drawn on screen based upon feature extraction done using image processing to identify the edges of campus road.
5. Object Classification using Deep Neural Net
6. Checking weather the object is in the path or not.
7. If in the path, then generate steering advice and update the speed based on the obstacle distance from the car.




















