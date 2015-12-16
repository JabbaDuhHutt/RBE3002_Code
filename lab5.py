#!/usr/bin/env python



import rospy

from std_msgs.msg import String

from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry

from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion

from tf.transformations import euler_from_quaternion

import tf

import numpy

import math 





#Kobuki Dimensions

wheel_rad  = 0.035  #m

wheel_base = 0.23 #m



rows=10
columns=10
fullmapGridExplored = [[-1 for x in range(rows)] for x in range(columns)]


def frontier(fullmapGridExplored):
	global currentPoint
	smallestCostCell = Point()
	smallestCostCell.x = 10000
	smallestCostCell.y = 10000
	smallestCostCell.z=0
	smallestDistance =10000
	for i in range(0,rows):
		for j in range(0,columns):
			amount =0
			if(fullmapGridExplored[i][j] > 0 and fullmapGridExplored[i][j]<9000):
				if(not i==0):
					if(fullmapGridExplored[i-1][j]==-1):
						amount= amount+1
				if(not j==0):
					if(fullmapGridExplored[i][j-1]==-1):
						amount= amount+1
				if(not i==columns):
					if(fullmapGridExplored[i+1][j]):
						amount= amount+1
				if(not j==rows):
					if(fullmapGridExplored[i][j+1]):
						amount= amount+1
				if(amount>0):
				        pass
					#put cells in arrayOfFrontires
	for element in arrayOfFrontires:#create array of frontiers at some point
		if(heuristic(currentPoint,element)<smallestDistance):
			smallestCostCell.x = element.x
			smallestCostCell.y = element.y
			smallestDistance = heuristic(currentPoint,element)
	localNav(smallestCostCell,currentPoint) #publish instead



#A *

def aSTAR(start,goal):

    global currentPoint

    global mapData

    global cells_met

    global cellThresh

    global doneFlag

    global cardinalDir
    global front
    global back
    global left
    global right

    cells_met = GridCells();

    cells_met.header.frame_id = 'map'

    cells_met.cell_width = 1 #m #change based off grid size

    cells_met.cell_height = 1 #m #change based off grid size

    #startPos = start.pose.position

    goalPos = goal.position

    readCurrentPos()

    #currentPoint = startPos #set up for rogers

    #publishObjectCells(mapData)

    while(not doneFlag and not rospy.is_shutdown()):

        costFront=9001

        costLeft=9001

        costRight=9001

        costBack=9001 

        hugh = heuristic(currentPoint,goalPos)

        print "HUGH:"

        print hugh

        if(hugh < cellThresh):

            doneFlag = True

            print "HUGH LOW"

            break

        mrRogers(currentPoint)

        if(not cellOccupied(front)):#im adding not cause I think I did logic wrong

            costFront=1

            costFront+=heuristic(front,goalPos)

        if(not cellOccupied(back)):

            costBack=1

            costBack+=heuristic(back,goalPos)

        if(not cellOccupied(left)):

            costLeft=1

            costLeft+=heuristic(left,goalPos)

        if(not cellOccupied(right)):

            costRight=1

            costRight+=heuristic(right,goalPos)

        if(costFront<costLeft):

            lowest=costFront

            direction= "front";

        else:

            lowest=costLeft

            direction="left"

        if(lowest>costBack):

            lowest=costBack

            direction="back"

        if(lowest>costRight):

            lowest=costRight

            direction="right"



        if(direction=="front"):

            Front()

            print "FORWARD"

        elif(direction=="right"):

            Right()

            print "RIGHT"

        elif(direction=="back"):

            Back()

            print "BACK"

        elif(direction=="left"):

            Left()

            print "LEFT"

        print "Current X:"

        print currentPoint.x

        print "Current Y:"

        print currentPoint.y

        print "Cardinal Dir:"

        print cardinalDir



    print "DONE"
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
def pointConversionToGmapping(pose):
    global origin
    
    newPose = Pose()
    newPose.position.x = pose.position.x + origin.position.x
    newPose.position.y = pose.position.y + origin.position.y
    newPose.orientation.z = pose.orientation.z + pose.orientation.z #i think this is right + over -
     
    #set cardinalDir based off start angle direction

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

    # #set to currentPoint
    # if((x - 0.25) < 0):
    #     correctX = round(x) + 0.25
    # else:
    #     correctX = round(x) - 0.25
    
    # if((y - 0.25) < 0):
    #     correctY = round(y) + 0.25
    # else:
    #     correctY = round(y) - 0.25

    currentPoint = Point(); #might need to change to pose if neighbors dont work out when looking different directions

    currentPoint.x = x

    currentPoint.y = y

    currentPoint.z = 0
#adjusts global neighbors to current neighbors

def mrRogers(current):

    #global pose #dont think this needs to be here

    #global currentPoint

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
#takes cell of point() and returns whether the cell is occupied or not

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

#currently heuristic is just straightline between point(start) and goal 

def heuristic(start,goal):

    global doneFlag



    

    distance = distanceFormula(start, goal)

    if(distance < cellThresh):

        print "I'm Here!!!"

        doneFlag=True

    

    return distance

#self explainatory but does distance formula on two points

def distanceFormula(start1,goal1):

    x0 = start1.x

    y0 = start1.y

    

    x1 = goal1.x

    y1 = goal1.y

    

    xx = x1-x0

    yy = y1-y0

    d = math.sqrt((math.pow(xx,2) + math.pow(yy,2)))

    return d

#progresses to right cell

def Right():

    global unit_cell

    global right

    global cells_met

    global cellPub

    global currentPoint

    global cardinalDir

    global goal

    global waypoints


    tempPoint = Point()
    tempPoint.x = right.x
    tempPoint.y = right.y
    tempPoint.z = right.z
    
    cells_met.cells.append(tempPoint)  
        

    waypoint = PoseStamped()

    T = len(cells_met.cells)

    if(T == 1):

        waypoint.header.frame_id = 'map'

        waypoint.pose.position.x = cells_met.cells[0].x

        waypoint.pose.position.y = cells_met.cells[0].y

        waypoint.pose.position.z = cells_met.cells[0].z
        
        waypoint.pose.orientation.z = 0 #until we think of better way but fine for now 
        waypoints.append(waypoint)
        
    elif((math.fabs(cells_met.cells[T-1].x - goal.position.x) < cellThresh) and (math.fabs(cells_met.cells[T-1].y - goal.position.y) < cellThresh) and (math.fabs(cells_met.cells[T-1].z - goal.position.z) < cellThresh)):
        waypoint.header.frame_id = 'map'

        waypoint.pose.position.x = cells_met.cells[T-1].x

        waypoint.pose.position.y = cells_met.cells[T-1].y

        waypoint.pose.position.z = cells_met.cells[T-1].z

        
        waypoint.pose.orientation.z = 0 #until we think of better way but fine for now 
        waypoints.append(waypoint)

  

    else:

        waypoint.header.frame_id = 'map'

        waypoint.pose.position.x = cells_met.cells[T-2].x

        waypoint.pose.position.y = cells_met.cells[T-2].y

        waypoint.pose.position.z = cells_met.cells[T-2].z

        

        waypoint.pose.orientation.z = 0 #until we think of better way but fine for now 
        waypoints.append(waypoint)
        

    #rotate(-1.571)

    #driveStraight(.2,unit_cell)

    

    currentPoint = right #tells us that we've moved on

    cardinalDir -= 1

    if(cardinalDir < 1):

        cardinalDir = 4

        



    cellPub.publish(cells_met)

#progresses to left cell

def Left():

    global cellPub

    global left

    global unit_cell

    global cells_met

    global currentPoint

    global cardinalDir

    global goal

    global waypoints

    

    tempPoint = Point()
    tempPoint.x = left.x
    tempPoint.y = left.y
    tempPoint.z = left.z
    
    cells_met.cells.append(tempPoint)  
    waypoint = PoseStamped()

    T = len(cells_met.cells)

    if(T == 1):

        waypoint.header.frame_id = 'map'

        waypoint.pose.position.x = cells_met.cells[0].x

        waypoint.pose.position.y = cells_met.cells[0].y

        waypoint.pose.position.z = cells_met.cells[0].z
        
        waypoint.pose.orientation.z = 0 #until we think of better way but fine for now
        
        waypoints.append(waypoint)
    elif((math.fabs(cells_met.cells[T-1].x - goal.position.x) < cellThresh) and (math.fabs(cells_met.cells[T-1].y - goal.position.y) < cellThresh) and (math.fabs(cells_met.cells[T-1].z - goal.position.z) < cellThresh)):
        waypoint.header.frame_id = 'map'

        waypoint.pose.position.x = cells_met.cells[T-1].x

        waypoint.pose.position.y = cells_met.cells[T-1].y

        waypoint.pose.position.z = cells_met.cells[T-1].z

        
        waypoint.pose.orientation.z = 0 #until we think of better way but fine for now
        waypoints.append(waypoint)
         

    else:

        waypoint.header.frame_id = 'map'

        waypoint.pose.position.x = cells_met.cells[T-2].x

        waypoint.pose.position.y = cells_met.cells[T-2].y

        waypoint.pose.position.z = cells_met.cells[T-2].z

        

        waypoint.pose.orientation.z = 0 #until we think of better way but fine for now
        waypoints.append(waypoint)     
           

    currentPoint = left #tells us that we've moved on

    cardinalDir += 1

    if(cardinalDir > 4):

        cardinalDir = 1

    #print currentPoint.x

    #print currentPoint.y

    #rotate(1.571)

    #driveStraight(.2,unit_cell)

    



    cellPub.publish(cells_met)

#progresses to front cell

def Front():

    global front

    global unit_cell

    global cellPub

    global cells_met

    global currentPoint

    global cardinalDir

    global goal

    global waypoints
    

    tempPoint = Point()
    tempPoint.x = front.x
    tempPoint.y = front.y
    tempPoint.z = front.z
    
    cells_met.cells.append(tempPoint)  
    waypoint = PoseStamped()

    T = len(cells_met.cells)
        

    if((math.fabs(cells_met.cells[T-1].x - goal.position.x) < cellThresh) and (math.fabs(cells_met.cells[T-1].y - goal.position.y) < cellThresh) and (math.fabs(cells_met.cells[T-1].z - goal.position.z) < cellThresh)):
        waypoint.header.frame_id = 'map'

        waypoint.pose.position.x = cells_met.cells[T-1].x

        waypoint.pose.position.y = cells_met.cells[T-1].y

        waypoint.pose.position.z = cells_met.cells[T-1].z

        
        waypoint.pose.orientation.z = 0 #until we think of better way but fine for now 
        waypoints.append(waypoint)
        

    currentPoint = front #tells us that we've moved on

    #print currentPoint.x

    #print currentPoint.y

    #driveStraight(.2,unit_cell)

    

    

    cellPub.publish(cells_met)

#progresses to back cell

def Back():

    global back

    global unit_cell

    global cells_met

    global cellPub

    global currentPoint

    global cardinalDir

    global goal

    global waypoints


    tempPoint = Point()
    tempPoint.x = back.x
    tempPoint.y = back.y
    tempPoint.z = back.z
    
    cells_met.cells.append(tempPoint)    

    waypoint = PoseStamped()

    T = len(cells_met.cells)

    if(T == 1):

        waypoint.header.frame_id = 'map'

        waypoint.pose.position.x = cells_met.cells[0].x

        waypoint.pose.position.y = cells_met.cells[0].y

        waypoint.pose.position.z = cells_met.cells[0].z
        
        waypoint.pose.orientation.z = 0 #until we think of better way but fine for now
        waypoints.append(waypoint)        
    elif((math.fabs(cells_met.cells[T-1].x - goal.position.x) < cellThresh) and (math.fabs(cells_met.cells[T-1].y - goal.position.y) < cellThresh) and (math.fabs(cells_met.cells[T-1].z - goal.position.z) < cellThresh)):
        waypoint.header.frame_id = 'map'

        waypoint.pose.position.x = cells_met.cells[T-1].x

        waypoint.pose.position.y = cells_met.cells[T-1].y

        waypoint.pose.position.z = cells_met.cells[T-1].z

        
        waypoint.pose.orientation.z = 0 #until we think of better way but fine for now 
        waypoints.append(waypoint)

  

    else:

        waypoint.header.frame_id = 'map'

        waypoint.pose.position.x = cells_met.cells[T-2].x

        waypoint.pose.position.y = cells_met.cells[T-2].y

        waypoint.pose.position.z = cells_met.cells[T-2].z

        

        waypoint.pose.orientation.z = 0 #until we think of better way but fine for now 
        waypoints.append(waypoint)

    

    currentPoint = back #tells us that we've moved on

    cardinalDir += 2

    if(cardinalDir == 5):

        cardinalDir = 1

    if(cardinalDir == 6):

        cardinalDir = 2

    #print currentPoint.x

    #print currentPoint.y    

    #rotate(3.14)

    #driveStraight(.2,unit_cell)



    cellPub.publish(cells_met)

#Odometry Callback function

def readOdom(msg):

    global pose

    global odom_tf



    pose = msg.pose

    geo_quat = pose.pose.orientation



    odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0), (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w), rospy.Time.now(), "base_footprint","odom")

def run():

    global mapData

    #global start

    global target

    global front

    global left

    global right

    global back

    global odom_tf

    global odom_list

    global pose

    global unit_cell

    global currentTheta #theta of current robot to map

    global currentPoint #might need to change to pose if neighbors dont work out

    global cardinalDir #direction robot is facing in respect to global map (1 is +y , 2 is -x, 3 is -y, 4 is +x)

    global occupiedCell #list of occupied cells
    global actual

    global initPos

    global currentAngle

    global origin

    global goal

    global doneFlag

    global threshHold

    global pub

    global pubGCell

    global unknownCell

    global cellPub

    global cellThresh

    global cells_met
    global width

    global height
    
    width = 0
    height = 0
    origin = Pose();

    unit_cell = 1 #m

    AMap = 0

    worldMap = 0

    path = 0

    mapData = 0
    

    currentAngle = 0 #radians

    cardinalDir = 1

    cellThresh = .2 #m

    threshHold = 3 #degrees?

    doneFlag = False

    front = Point();

    left = Point();

    right = Point(); 

    back = Point(); #might need to change depending on cells

    

    initPos = PoseStamped();

    

    rospy.init_node('lab3')

    

    #subscribers

    worldMapSub = rospy.Subscriber('/map', OccupancyGrid, readWorldMap)

    markerSub = rospy.Subscriber('/move_base_simple/goalRBE', PoseStamped, readGoal)

    odomSub = rospy.Subscriber('/odom', Odometry, readOdom)

    sub = rospy.Subscriber('/initialPose', PoseWithCovarianceStamped, startCallBack)

    

    #publishers

    pubGCell = rospy.Publisher('/grid_check', GridCells, queue_size=1)

    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)

    cellPub = rospy.Publisher('/cell_path', GridCells, queue_size = 100)

    pathPub = rospy.Publisher('/path_path', Path, queue_size = 1)

    unknownCell = rospy.Publisher('/grid_unknown', GridCells, queue_size=1)

    

    #listener/broadcaster might not be needed but here

    odom_list = tf.TransformListener()

    odom_tf = tf.TransformBroadcaster()

    

    odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"map","base_footprint")

    

    sleeper = rospy.Duration(1)

    rospy.sleep(sleeper)

    

    target = 0

    start = 0

    end = 0



    while not rospy.is_shutdown():

        print("starting")

        #odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"map","base_footprint")

        publishObjectCells(mapData)
        #testPoint = Point()
        #testPoint.x = 0
        #testPoint.y = 3
        #testPoint.z = 0
        
        #x = actualOccupiedCells.cells
        #print x
        

        #driveStraight(.2, 1)

        print("waiting")

        rospy.loginfo("Waiting...")

        rospy.spin() 

        #rospy.is_shutdown()







    

if __name__ == '__main__':

    try:

        run()

    except rospy.ROSInterruptException:

        pass