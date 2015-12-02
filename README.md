README.md for Fall 2015 Introduction to Robotics. 


Assignment: Programming Assignment 3
Class: ROB 514 Intro to Rob
Date: 12/01/15
Author: Kory Kraft


scikit for nearest neighbors clustier in required.
sudo pip install -U scikit-learn


Purpose: 
The robot drives around and places markers on the circular objects in the world (from an overhead 2d perspective).





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


