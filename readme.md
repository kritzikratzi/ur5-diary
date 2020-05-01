# UR5 - Diary


This repository contains a series of small tests with the universal robots UR-5 robot. 
The tests might not be immediately runnable on your computer without a bit of effort. On top of that the code isn't really cleaned up. It's more or less a personal history while trying to figure how to use the robot and what to do with it. 


The code is written either in Python or C++, utilizing the following libraries: 

* python-urx to drive the robot from python https://github.com/jkur/python-urx.git
* ur_rtde to drive the robot from python/c++ https://gitlab.com/sdurobotics/ur_rtde
* cmake to build c++ projects https://cmake.org/
* glm for vector math in c++ https://glm.g-truc.net
* numpy and scipy for vector maths in python https://numpy.org/ https://www.scipy.org/
* opencv for image recognition https://opencv.org/
* gphoto2 for camera tethering http://www.gphoto.org/



# Setting up python-urx

I use the fork from jkur, install it using:

	pip3 install git+https://github.com/jkur/python-urx.git

## Most basic example: 


	import urx
	
	
	a = 0.1
	v = 0.1


	rob = urx.Robot("192.168.0.100"); 
	rob.movel((+0.1,-0.5,0.05, 0, 0, 0), a, v)
	rob.movel((+0.1,-0.5,0.00, 0, 0, 0), a, v)
	rob.movel((-0.1,-0.5,0.00, 0, 0, 0), a, v)
	rob.movel((-0.1,-0.5,0.05, 0, 0, 0), a, v)
	rob.close()
	
	
# Setting up ur_rtde

ur_rtde is a library for both python and C++. 

The library itself is great, but unfortunately pybind11 dependency was acting strange on my computer. For me this was the setup process: 

	pip3 install --user pybind11
	cd /usr/local/include/python3.7m
	ln -s /Users/hansi/Library/Python/3.7/include/python3.7m/pybind11 pybind11


You can figure out where python3 thinks pybind11 should be available by running
	
	run python3, then: 
	
	import pybind11
	pybind11.get_include()
	pybind11.get_include(True)
	

This lists two paths, in my case `/usr/local/include/python3.7m/pybind11` and `/Users/hansi/Library/Python/3.7/include/python3.7m/pybind11`. Make sure the header files are present in both folders! 


Once this is settled you can run 

	pip3 install ur_rtde



# List of Examples

## test_1 (python)

Uses python and python-urx and does simple motions only. There are three examples inside: 

* test_1: Moves the robot to different points
* test_2: Draws a grid
* test_3: Draws the xy-representation of a stereo audio file

## test_2 (python/C++)

* test_2_1: Moves the tool point to different orientations using python-urx
* test_2_2: Moves to different points in space using the python binds of ur_rtde

## test_2_servoj (C++)

Uses C++, cmake and ur_rtde to make smooth robot motions. 

## test_3_crooked_pen

Uses C++, cmake and ur_rtde to draw with a swerving pen motion attempting to resemble the motion of a human hand. 
The basis for the drawings are again stereo audio files interpreted as xy-curves. 



## test_4_force (C++)

Moves the robot up, then lets the arm fall freely until it hits the ground. 
This could be used to quickly measure the length of a pen the gripper is holding. 

## test_5_poke (C++)

Testing to see if it's possible to "poke" a human using the robot arm. I consider this experiment an utter failure. 

## test_6_recplay (C++)

Puts the robot into freedrive. It then records data points until the human stops moving the robot. After that the motion is replayed. 

## test_7_snake (C++)

An attempt to make robot move in the way a snake would. I consider this another undisputable failure. 

## test_8_camera (C++) 

Attached a standard SLR camera to the robot arm using a 3d printed tool adapter. gphoto2 is used for tethering. 

## test_9_gummi (ongoing! python/C++)

Here gummi bears are picked and placed. The gummi bears are detected using opencv, Charuco markers are used to align the opencv camera view 
to the robot coordinate system. 

The C++ part handles image recognition and coordinate conversion. Python access the coordinates using a rest interface. 
Very much in progress. 


## test_10_kresse (ongoing! C++)

Very much in progress. This is a continuation of test_8_camera, but for the laowa macro probe lense and ... much more complicated of course. 

