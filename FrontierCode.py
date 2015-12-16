#!/usr/bin/env python



import rospy

from std_msgs.msg import String

from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry

from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion

from tf.transformations import euler_from_quaternion

import tf

import numpy

import math 

#debug github
#intializes search by taking in current position
def searchFrontiers():
	global currentPoint
	#global cardinalDir
	global front
	global right
	global left
	global back
	global visited
	global frontier_flag
	global frontier_list

	#cardinalDir = 1

	frontier_list = []

#move current point to local
	readCurrentPos()
	currentPoint.x = cx
	currentPoint.y = cy
#create local point for start
	startPoint = Point();
	startPoint.x = cx
	startPoint.y = cy

	visited = []
	frontier_flag = []

	bfs = Queue()
	bfs.put(startPoint)

	while (not bfs.empty()):
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

		

		for x in nbr:
			t = len(visited)
			if(((not cellOccupied(x)) and (not unknownSpace(x))) and  not visited(x, t)):
				visited.append(x) #marks cell as visited
				bfs.put(x) #places in search so we can look at its surroundings
			elif(isNewFrontierCell(x)):
				frontier_flag.append(x) #marks cell as frontier
				new_frontier = buildNewFrontier(x)
				if(new_frontier.size > 1):
					n = len(frontier_list) - 1
					frontier_list.insert(n, new_frontier) #puts frontier at the end of the list

	return frontier_list #might not do this because node might just shot accros but will do for now


#returns true if the cell is a new frontier cell
def isNewFrontierCell(cell):
	global front
	global back
	global right
	global left
	global currentPoint

	readCurrentPos()

	if(not unknownSpace(cell) or frontier_flag(cell)):
		return False
	
	#make list of neighbor to currentPoint
	nbr = []
	nbr.append(fx)
	nbr.append(rx)
	nbr.append(lx)
	nbr.append(bx)

	for x in nbr:
		if(unknownSpace(x)):
			return True

	return False
#builds new frontier based on given cell
def buildNewFrontier(cell):



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

    #if(cardinalDir == 1):

    front.x = x

    front.y = y + unit_cell

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

    # elif(cardinalDir == 2):

    #     front.x = x - unit_cell

    #     front.y = y

    #     front.z = 0

    #     print "front neighbor found"

    #     left.x = x

    #     left.y = y

    #     left.y -= unit_cell

    #     left.z = 0

    #     print "left neighbor found"

    #     back.x = x + unit_cell

    #     back.y = y 

    #     back.z = 0

    #     print "back neighbor found"

    #     right.x = x

    #     right.y = y + unit_cell

    #     right.z = 0

    #     print "right neighbor found"

    # elif(cardinalDir == 3):

    #     front.x = x

    #     front.y = y - unit_cell

    #     front.z = 0

    #     print "front neighbor found"

    #     left.x = x + unit_cell

    #     left.y = y

    #     left.z = 0

    #     print "left neighbor found"

    #     back.x = x

    #     back.y = y + unit_cell

    #     back.z = 0

    #     print "back neighbor found"

    #     right.x = x - unit_cell

    #     right.y = y

    #     right.z = 0

    #     print "right neighbor found"

    # else:

    #     front.x = x + unit_cell

    #     front.y = y

    #     front.z = 0

    #     print "front neighbor found"

    #     left.x = x

    #     left.y = y + unit_cell

    #     left.z = 0

    #     print "left neighbor found"

    #     back.x = x - unit_cell

    #     back.y = y 

    #     back.z = 0

    #     print "back neighbor found"

    #     right.x = x

    #     right.y = y - unit_cell

    #     right.z = 0

    #     print "right neighbor found"
#checks to see if cell is occupied
def cellOccupied(cell):
    global occupiedCells

    global cellThresh

    global actualOccupiedCells

    #for each occupiedCell compare the point to point that was passed in

    for occupied in actualOccupiedCells.cells:

        if((math.fabs(occupied.x - cell.x) < cellThresh) and (math.fabs(occupied.y - cell.y) < cellThresh) and (math.fabs(occupied.z - cell.z) < cellThresh)): #break up for debug if not equating 

            return True

        else:

            return False
# param: cell is used to tell if it in the visited list
# param: lenght is used to tell if visited is empty and if so just return false
def visited(cell, length):
    global visited


    #check visited for "cell"
    #if cell has been visited return true
    if(length <= 0):
    	for occupied in visited:

	        if((occupied.x == cell.x) and (occupied.y == cell.y) and (occupied.z == cell.z)):

	            return True

	        else:

	            return False
	else:
		return False
#takes a cell and determines if it has been marked as a frontier_flag
def frontier_flag(cell):
	global frontier_flag


    #check frontier_flag for "cell"
    #if cell has been marked as a frontier return true

    for frontier in frontier_flag:

        if((frontier.x == cell.x) and (frontier.y == cell.y) and (frontier.z == cell.z)):

            return True

        else:

            return False
#checks to see if cell unknown
def unknownSpace(cell):
	global UnknownSpace

	for unknown in UnknownSpace.cells:
		if((math.fabs(unknown.x - cell.x) < cellThresh) and (math.fabs(unknown.y - cell.y) < cellThresh) and (math.fabs(unknown.z - cell.z) < cellThresh)): #break up for debug if not equating 

            return True

        else:

            return False
	
#publish or just get grid cells that have obstacle with x,y location
#FIXED
def publishObjectCells(grid):
    global pubGCell
    #global unknownCell
    global UnknownSpace
    global height
    global width
    global occupiedCells
    global actualOccupiedCells
    k = 0
    b = 0
    actualOccupiedCells = GridCells()
    actualOccupiedCells.header.frame_id = 'local'
    actualOccupiedCells.cell_width = 1
    actualOccupiedCells.cell_height = 1
    occupiedCells = GridCells()
    occupiedCells.header.frame_id = 'map'
    occupiedCells.cell_width = 0.05 #change based off grid size
    occupiedCells.cell_height = 0.05 #change based off grid size
    
    for i in range(1,height): #height should be set to hieght of grid
        k=k+1
        for j in range(1,width): #height should be set to hieght of grid
            k=k+1
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=j*.05 # edit for grid size
                point.y=i*.05 # edit for grid size
                point.z=0
                occupiedCells.cells.append(point)
    
    pubGCell.publish(occupiedCells)

    for i in range(0, len(occupiedCells.cells) - 1):
        tempCell = Point()
        tempCell.x=int((occupiedCells.cells[i].x/20))
        tempCell.y=int((occupiedCells.cells[i].y/20))
        tempCell.z=0
        
        
        if(tempCell not in actualOccupiedCells.cells):
            actualOccupiedCells.cells.append(tempCell)
    
    
    Cells = GridCells()
    Cells.header.frame_id = 'map'
    Cells.cell_width = 0.05 #change based off grid size
    Cells.cell_height = 0.05 #change based off grid size
    UnknownSpace = GridCells()
    UnknownSpace.header.frame_id = 'map'
    UnknownSpace.cell_width = 1 #m
    UnknownSpace.cell_height = 1 #m
    
    for i in range(1,height): #height should be set to hieght of grid

        b=b+1
        for j in range(1,width): #height should be set to hieght of grid
            b=b+1
            #print b # used for debugging
            #-1 means unknown
            if (grid[b] == -1):
                point1=Point()
                point1.x=j*.05 # edit for grid size
                point1.y=i*.05 # edit for grid size
                point1.z=0
                Cells.cells.append(point1)
    
    for i in range(0, len(Cells.cells) - 1):
    	unknown = Point()
    	unknown.x = int((Cells.cells[i].x/20))
    	unknown.y = int((Cells.cells[i].y/20))
    	unknown.z = 0

    	if(tempCell not in ):
            UnknownSpace.cells.append(unknown)
    #unknownCell.publish(Cells)