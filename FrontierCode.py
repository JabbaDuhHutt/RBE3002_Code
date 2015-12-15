#!/usr/bin/env python



import rospy

from std_msgs.msg import String

from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry

from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion

from tf.transformations import euler_from_quaternion

import tf

import numpy

import math 

#intializes search by taking in current position
def initializeFrontiers():
	global currentPoint
	global front
	global right
	global left
	global back
	global visited
	global frontier_flag

#move current point to local
	readCurrentPos()
	currentPoint.x = cx
	currentPoint.y = cy
#create local point for start
	startPoint = Point();
	startPoint.x = cx
	startPoint.y = cy

	bfs = Queue()
	bfs.put(startPoint)
	while (!bfs.empty()):
		#sets current point of iterations and removes from queue
		point = bfs.get()
		#run get nbhood4 for currentPoint
		mrRogers(point)
		#make list of neighbor to currentPoint
		fx = front
		rx = right
		lx = left
		bx = back
		#make list of neighbor to currentPoint
		nbr = []
		nbr.append(fx)
		nbr.append(rx)
		nbr.append(lx)
		nbr.append(bx)

		visted = []

		for x in nbr:
			t = len(visted)
			if(!cellOccupied(x) and !visted()){
				
			}


def pointConversionToGmapping(pose):
    global origin
    
    newPose = Pose()
    newPose.position.x = pose.position.x + origin.position.x
    newPose.position.y = pose.position.y + origin.position.y
    newPose.orientation.z = pose.orientation.z + pose.orientation.z #i think this is right + over -
     
def pointConversionToReferrence(pose):
    global origin
    
    newPose = Pose()
    newPose.position.x = pose.position.x - origin.position.x
    newPose.position.y = pose.position.y - origin.position.y
    newPose.orientation.z = pose.orientation.z - pose.orientation.z #i think this is right + over -
#read map data

def readWorldMap(data):

    # map listener

    global mapData, grid

    global width

    global height
    global origin
    
    origin = data.info.origin

    grid = data

    mapData = data.data

    width = data.info.width

    height = data.info.height

def readCurrentPos():

    global pose

    global currentPoint

    global cardinalDir

    global threshHold

    

    pose = Pose();

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))

    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))

    x=position[0]#position.pose.pose.position.x

    y=position[1]#position.pose.pose.position.y

    odomWX=orientation[0]

    odomWY=orientation[1]

    odomWZ=orientation[2]

    odomWW=orientation[3]

    q = [odomWX,odomWY,odomWZ,odomWW]

    #odomW = orientation.pose.pose.orientation

    #q = [odomW.x, odomW.y, odomW.z, odomW.w]###############################################################################

    roll, pitch, yaw = euler_from_quaternion(q)

    #convert yaw to degrees

    global currentAngle

    #print currentTheta

    currentAngle = yaw #to correct for driving issues

    currentTheta = math.degrees(yaw)

    if(math.fabs(0 - currentTheta) < threshHold): #might need to do negative angle and positive (also might want <=)

        cardinalDir = 4

    elif(math.fabs(90 - currentTheta) < threshHold):

        cardinalDir = 1

    elif(math.fabs(180 - currentTheta) < threshHold):

        cardinalDir = 2

    elif(math.fabs(270 - currentTheta) < threshHold):

        cardinalDir = 3

    else:

        print "Angle off or threshHold too low"

    #set to currentPoint
    #disregard was for when not using move_stack
    # if((x - 0.25) < 0):
    #     correctX = round(x) + 0.25
    # else:
    #     correctX = round(x) - 0.25
    
    # if((y - 0.25) < 0):
    #     correctY = round(y) + 0.25
    # else:
        # correctY = round(y) - 0.25

    currentPoint = Point(); #might need to change to pose if neighbors dont work out when looking different directions

    currentPoint.x = x

    currentPoint.y = y

    currentPoint.z = 0

#generates nbhood4 for "current"
def mrRogers(current):

    global front

    global left

    global right

    global back

    global unit_cell

    global cardinalDir

    global height

    global width

    

    x = current.x

    y = current.y

    print "Will you be my neighbor"

    if(cardinalDir == 1):

        front.x = y

        front.y = x + unit_cell

        front.z = 0

        print "front neighbor found"

        left.x = x - unit_cell #if point gets negative then off map do something to deal with this

        left.y = y

        left.z = 0

        print "left neighbor found"

        back.x = x

        back.y = y - unit_cell

        back.z = 0

        print "back neighbor found"

        right.x = x + unit_cell

        right.y = y

        right.z = 0

        print "right neighbor found"

    elif(cardinalDir == 2):

        front.x = x - unit_cell

        front.y = y

        front.z = 0

        print "front neighbor found"

        left.x = x

        left.y = y

        left.y -= unit_cell

        left.z = 0

        print "left neighbor found"

        back.x = x + unit_cell

        back.y = y 

        back.z = 0

        print "back neighbor found"

        right.x = x

        right.y = y + unit_cell

        right.z = 0

        print "right neighbor found"

    elif(cardinalDir == 3):

        front.x = x

        front.y = y - unit_cell

        front.z = 0

        print "front neighbor found"

        left.x = x + unit_cell

        left.y = y

        left.z = 0

        print "left neighbor found"

        back.x = x

        back.y = y + unit_cell

        back.z = 0

        print "back neighbor found"

        right.x = x - unit_cell

        right.y = y

        right.z = 0

        print "right neighbor found"

    else:

        front.x = x + unit_cell

        front.y = y

        front.z = 0

        print "front neighbor found"

        left.x = x

        left.y = y + unit_cell

        left.z = 0

        print "left neighbor found"

        back.x = x - unit_cell

        back.y = y 

        back.z = 0

        print "back neighbor found"

        right.x = x

        right.y = y - unit_cell

        right.z = 0

        print "right neighbor found"