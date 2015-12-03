README.md for Fall 2015 Introduction to Robotics. 


Assignment: Programming Assignment 3
Class: ROB 514 Intro to Rob
Date: 12/01/15
Author: Kory Kraft

*********************************
**** Required dependencies *******
**********************************

scikit and scipy for nearest neighbors clustier is required.

  sudo pip install -U scikit-learn
  sudo apt-get install python-numpy python-scipy 

ros frontier exploration also required (aside from pre installed turtlebot stage)
  
  sudo apt-get install ros-indigo-frontier-exploration

*********************************
***** Purpose *******************
*********************************

The robot drives around and places markers on the circular objects in the world (from an overhead 2d perspective).

********************************************
***** Launching Instructions ***************
*********************************************

To teleop the robot using standard teleop controls:

  terminal 1) roslaunch exercise1 teleop_circle_detection.launch
  terminal 2) roslaunch turtlebot_teleop keyboard_teleop.launch

To use frontier exploration:
  
  roslaunch exercise1 auto_circle_detection.launch

This will then require the user to select a bounded polygon region for the robot using the publish point tool and a starting location for the robot. See http://wiki.ros.org/frontier_exploration for more details

*********************************
**** Known Issues ***************
*********************************

There are many false positives for circles. This is in part due to the weak circle requirement definitions in my fitting code.
Also, the markers are not persistent. I was working on that (see the note at the top of circle_detector.py in the src folder) but I decided to call it "good enough."

Having to draw the bounded box is a pain, but functional. This could/should be updated.












Assignment: Programming Assignment 2
Class: ROB 514 Intro to Rob
Date: 11/18/2015
Author: Kory Kraft

This package includes all the contents for exercise 2 and 1.

Tor run the stuff related to programming assignment 2, you must have espeak installed!!!!! To do this, run
	sudo apt-get install espeak

Next, turn your speakers up and launch the appropriate launch file with the cmd:
	roslaunch exercise1 auto_driver.launch

To kill the program, press Esc on the new terminal or just click the x button on it. Everything else will shutdown.





Assignment: Exercise 1
Class: ROB 514 Intro to Rob
Date: 10/21/2015
Author: Kory Kraft

This package includes all the contents for exercise 1.

3 launch files are included in the package. Each of them starts the turtlebot_stage simulator. This is a required package.

1)
   roslaunch exercise1 stopper.launch

   This command launches the simple version of the code that runs the simulator and drives the robot forward until it reaches a specified distance from an obstacle.

2) 

   roslaunch exercise1 driver.launch

   This command launches the version 1 guarded teleop code. The user is able to teleop the robot around with the keyboard, but the guarding node protects the robot from running into a wall.

3) 

   roslaunch exercise1 driver_v2.launch

   This command launches version 2 of the extended guarded teleop code that allows the user to return the robot to its home location.

   To return the robot to it's home, press the 'h' key. Other teleop control keys are normal.

   The home x and y coordinates can be specified with the respecitive parameters in the launch file (or using the rosparam set commandline tool).

   Note, it takes 2 Ctrl-C's to kill the program (due to how keyboard input is read).


