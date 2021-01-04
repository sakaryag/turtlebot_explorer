#!/usr/bin/env python
##
##GORKEM SAKARYA
## 3 New Functions are created:
## -robot_move
##      That function takes 2 arguments and return motor_command
##      According to data input function decides to its move. And returns its
##      movements within motor_command to the wall_finding or wall_following functions
##
## -wall_finding
##      That function aims to find a wall to follow it. In this purpose robot goes forward
##      when it is close to wall it makes the wall finding flag and starts to follow
## -wall_following
##      That function implementation makes robot able to follow the wall. Robot follows the
##      the wall on the right. It keeps going with some go right and go left movements for
##      follow it. I implement all possible solutions for the cases that:
##      -isnan   -bigger than upper bound =1  -lower that upper bound      =0.5
##      Also there are some more cases are implemented for the cases that:
##      -bigger that lower bound     -lower that lower bound
##
##
##
##
##
##

import rospy
## Required for some printing options
import sys

## This is needed for the data structure containing the motor command.
from geometry_msgs.msg import Twist
## This is needed for the data structure containing the laser scan
from sensor_msgs.msg import LaserScan
## This is needed for the data structure containing the map (which you may not use).
from nav_msgs.msg import OccupancyGrid

## The following function is a "callback" function that is called back whenever a new laser scan is available.
## That is, this function will be called for every new laser scan.
##
## --------------------------------------------------------------------------------
## ----------CHANGE THIS FUNCTION TO MAKE THE ROBOT EXPLORE INTELLIGENTLY----------
## --------------------------------------------------------------------------------
#

global counter
counter=0
global find_wall
find_wall=True

global error_right
error_right=0

global error_left
error_left=0
global error_forward
error_forward=0

from datetime import datetime, timedelta
from math import pi, isnan

def robot_move(move_type,motor_command):
    global error_left
    error_left=error_left
    global error_right
    error_right=error_right
    global error_forward
    error_forward=error_forward
    if (move_type == "STOP") :
        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.0;

    elif (move_type == "FORWARD") :
        print '[ROBOT] GO FORWARD! '
	error_left=0
        error_right=0
	error_forward = error_forward+1
        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.5;


    elif (move_type == "BACKWARD") :
	print '[ROBOT] GO BACKWARD! '
	error_left=0
        error_right=0
	error_forward=0
        motor_command.linear.x = -0.5;
        motor_command.angular.z = 0.0;


    elif (move_type =="TURN_LEFT") :
        print '[ROBOT] Turn left! '
	error_left=error_left+1
        error_right=0
	error_forward=0
        motor_command.linear.x = 0.0;
        motor_command.angular.z = 1.5;
    elif (move_type =="ERROR_LEFT") :
        print '[ROBOT] RUN AWAY FROM ERROR! '
	error_left=0
	error_right=0
	error_forward=0
        motor_command.linear.x = -0.5;
        motor_command.angular.z = 9;
    elif (move_type =="ERROR_RIGHT") :
        print '[ROBOT] RUN AWAY FROM ERROR! '
	error_left=0
	error_right=0
	error_forward=0
        motor_command.linear.x =-0.5;
        motor_command.angular.z = -9;


    elif (move_type == "TURN_RIGHT") :
        print '[ROBOT] Turn Right! '
	error_left=0
        error_right=error_right+1
	error_forward=0
        motor_command.linear.x = 0.0;
        motor_command.angular.z = -1.5;

    elif (move_type == "GO_RIGHT") :
        print '[ROBOT] Go Right! '
	error_left=0
        error_right=error_right+1
	error_forward=0

        motor_command.linear.x = 0.15;
        motor_command.angular.z = -0.15;

    elif (move_type == "GO_LEFT") :
        print '[ROBOT] Go left! '
	error_left=error_left+1
        error_right=0
	error_forward=0
        motor_command.linear.x =  0.15;
        motor_command.angular.z = 0.15;



    return motor_command
def wall_finding(data,motor_command):
	middle=data.ranges[len(data.ranges)/2]
        left= data.ranges[-1]
        right =  data.ranges[0]
	global find_wall
        find_wall=find_wall

        small = 0.5
        big =1

	if  right<=big or left<=big or middle<=big:
	    	find_wall=False
	else:
	   	motor_command=robot_move("FORWARD",motor_command)
	return motor_command

def wall_following(data,motor_command):
	middle=data.ranges[len(data.ranges)/2]
        left= data.ranges[-1]
        right =  data.ranges[0]

        small = 0.5
        big =1

	if isnan(right) and isnan(left) and isnan(middle):
	    	    motor_command=robot_move("GO_RIGHT",motor_command)
	elif isnan(right) and isnan(left) and middle>big:
		    motor_command=robot_move("GO_RIGHT",motor_command)
	elif isnan(right) and isnan(left) and middle<=big:
		if middle>small:
			motor_command=robot_move("GO_RIGHT",motor_command)
		else:
			motor_command=robot_move("TURN_RIGHT",motor_command)
#-----------------------------------------------------------------------
	elif isnan(right) and left>big and isnan(middle):
	    	    motor_command=robot_move("GO_RIGHT",motor_command)
	elif isnan(right) and left>big and middle>big:
		    motor_command=robot_move("GO_RIGHT",motor_command)
	elif isnan(right) and left>big and middle<=big:
		if middle>small:
			motor_command=robot_move("GO_RIGHT",motor_command)
		else:
			motor_command=robot_move("TURN_RIGHT",motor_command)
#-----------------------------------------------------------------------
	elif isnan(right) and left<=big and isnan(middle):
		if left>small:
			motor_command=robot_move("GO_RIGHT",motor_command)
		else:
			motor_command=robot_move("TURN_RIGHT",motor_command)
	elif isnan(right) and left<=big and middle>big:
		if left>small:
			motor_command=robot_move("GO_RIGHT",motor_command)
		else:
			motor_command=robot_move("TURN_RIGHT",motor_command)
	elif isnan(right) and left<=big and middle<=big:
		motor_command=robot_move("TURN_LEFT",motor_command)


########################################################################
	elif right>big and isnan(left) and isnan(middle):
	    	    motor_command=robot_move("GO_RIGHT",motor_command)
	elif right>big  and isnan(left) and middle>big:
		    motor_command=robot_move("GO_RIGHT",motor_command)
	elif right>big  and isnan(left) and middle<=big:
		if middle>small:
			motor_command=robot_move("GO_RIGHT",motor_command)
		else:
			motor_command=robot_move("TURN_RIGHT",motor_command)
#-----------------------------------------------------------------------
	elif right>big  and left>big and isnan(middle):
	    	   motor_command=robot_move("GO_RIGHT",motor_command)
	elif right>big  and left>big and middle>big:
		    motor_command=robot_move("GO_RIGHT",motor_command)
	elif right>big  and left>big and middle<=big:
		if middle>small:
			motor_command=robot_move("GO_LEFT",motor_command)
		else:
			motor_command=robot_move("TURN_LEFT",motor_command)
#-----------------------------------------------------------------------
	elif right>big  and left<=big and isnan(middle):
	    	if left>small:
			motor_command=robot_move("GO_RIGHT",motor_command)
		else:
			motor_command=robot_move("TURN_RIGHT",motor_command)
	elif right>big  and left<=big and middle>big:
		    motor_command=robot_move("GO_RIGHT",motor_command)
	elif right>big  and left<=big and middle<=big:

		    motor_command=robot_move("TURN_LEFT",motor_command)

########################################################################
	elif right<=big and isnan(left) and isnan(middle):
	    	if right>small:
			motor_command=robot_move("GO_LEFT",motor_command)
		else:
			motor_command=robot_move("TURN_LEFT",motor_command)
	elif right<=big and isnan(left) and middle>big:
		    motor_command=robot_move("GO_LEFT",motor_command)
	elif right<=big and isnan(left) and middle<=big:
		if right<=small:
			motor_command=robot_move("TURN_LEFT",motor_command)
		else:
			if middle>small:
				motor_command=robot_move("GO_LEFT",motor_command)
			else:
				motor_command=robot_move("TURN_LEFT",motor_command)
#-----------------------------------------------------------------------
	elif right<=big and left>big and isnan(middle):
		if right>small:
			motor_command=robot_move("GO_LEFT",motor_command)
		else:
			motor_command=robot_move("TURN_LEFT",motor_command)
	elif right<=big and left>big and middle>big:
		        motor_command=robot_move("GO_LEFT",motor_command)
	elif right<=big and left>big and middle<=big:
		if right<=small:
			motor_command=robot_move("TURN_LEFT",motor_command)
		else:
			if middle>small:
				motor_command=robot_move("GO_LEFT",motor_command)
			else:
				motor_command=robot_move("TURN_LEFT",motor_command)
#-----------------------------------------------------------------------
	elif right<=big and left<=big and isnan(middle):
	    	if right>small:
			motor_command=robot_move("GO_LEFT",motor_command)
		else:
			motor_command=robot_move("TURN_LEFT",motor_command)
	elif right<=big and left<=big and middle>big:
		if right>small:
			motor_command=robot_move("GO_LEFT",motor_command)
		else:
			motor_command=robot_move("TURN_LEFT",motor_command)
	elif right<=big and left<=big and middle<=big:
		motor_command=robot_move("TURN_LEFT",motor_command)
	return motor_command

def laser_callback(data):
	motor_command = Twist()
	global error_right
	error_right=error_right
	middle=data.ranges[len(data.ranges)/2]
        left= data.ranges[-1]
        right =  data.ranges[0]
	global error_left
	error_left=error_left
	global error_forward
	error_forward=error_forward

        global find_wall
        find_wall=find_wall
        if(error_left>200 and  isnan(left)):
		motor_command=robot_move("ERROR_LEFT",motor_command)
		motor_command_publisher.publish(motor_command);

	elif(error_right>200 and  isnan(right)):
		motor_command=robot_move("ERROR_RIGHT",motor_command)
   		motor_command_publisher.publish(motor_command);

	elif(error_forward>50 and  isnan(middle)):
		motor_command=robot_move("ERROR_LEFT",motor_command)

   		motor_command_publisher.publish(motor_command);

	elif find_wall:
		motor_command=wall_finding(data,motor_command)
	   	motor_command_publisher.publish(motor_command);
	else:
		motor_command=wall_following(data,motor_command)
	   	motor_command_publisher.publish(motor_command);


    ## Alternatively we could have looked at the laser scan BEFORE we made this decision
    ## Well Lets see how we might use a laser scan
    ## Laser scan is an array of distances
	print 'Number of points in laser scan is: ', len(data.ranges)
	print 'The distance to the rightmost scanned point is: ', data.ranges[0]
	print 'The distance to the leftmost scanned point is: ', data.ranges[-1]
	print 'The distance to the middle scanned point is: ', data.ranges[len(data.ranges)/2]
	    ## You can use basic trigonometry with the above scan array and the following information to find out exactly where the laser scan found something
	print 'The minimum angle scanned by the laser is: ', data.angle_min
	print 'The maximum angle scanned by the laser is: ', data.angle_max
	print 'The increment in the angles scanned by the laser is: ', data.angle_increment
	    ## angle_max = angle_min+angle_increment*len(data.ranges)
	print 'The minimum range (distance) the laser can perceive is: ', data.range_min
	print 'The maximum range (distance) the laser can perceive is: ', data.range_max

## You can also make use of the map which is being built by the "gslam_mapping" subsystem
## There is some code here to help but you can understand the API also by looking up the OccupancyGrid message and its members (this is the API for the message)
## If you want me to explain the data structure, I will - just ask me in advance of class
def map_callback(data):
    chatty_map = False
    if chatty_map:
        print "-------MAP---------"
        ## Here x and y has been incremented with five to make it fit in the terminal
        ## Note that we have lost some map information by shrinking the data
        for x in range(0,data.info.width-1,5):
            for y in range(0,data.info.height-1,5):
                index = x+y*data.info.width
                if data.data[index] > 50:
                    ## This square is occupied
                    sys.stdout.write('X')
                elif data.data[index] >= 0:
                    ## This square is unoccupied
                    sys.stdout.write(' ')
                else:
                    sys.stdout.write('?')
            sys.stdout.write('\n')
        sys.stdout.flush()
        print "-------------------"

## This is the method we initilize everything
def explorer_node():
    ## We must always do this when starting a ROS node - and it should be the first thing to happen
    rospy.init_node('amble')

    ## Here we declare that we are going to publish "Twist" messages to the topic /cmd_vel_mux/navi. It is defined as global because we are going to use this publisher in the laser_callback.
    global motor_command_publisher
    motor_command_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)

    ## Here we set the function laser_callback to recieve new laser messages when they arrive
    rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size = 1000)

    ## Here we set the function map_callback to recieve new map messages when they arrive from the mapping subsystem
    rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size = 1000)

    ## spin is an infinite loop but it lets callbacks to be called when a new data available. That means spin keeps this node not terminated and run the callback when nessessary.
    rospy.spin()

if __name__ == '__main__':
    explorer_node()
