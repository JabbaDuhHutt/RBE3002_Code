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



def fullDrive():#suppose to take in an array of waypoints and navigate
    global currentPoint
    #global done
    global aFlag
    global waypoints
    tempArrays=[]
    print waypoints[0]
    for i in range (0,len(waypoints)):
        print "HERE"
        tempArrays.append(waypoints[i])
        
    waypoints=[]
    aFlag=False
    for element  in tempArrays:
        readCurrentPos()
        if(aFlag):
	        aSTAR(currentPoint,element.pose)
	        aFlag=False
	        fullDrive()
        localNav(element,currentPoint)
	print("All Way Points Reached!!!")



def localNav(waypoint,currentPos):#suppose to do a local A* but I'm not sure how the Slam gmapping works
	#global done
	global aFlag
	#print "Here"
	stuck = False
	while not ((math.fabs(currentPos.x - waypoint.pose.position.x) < cellThresh) and (math.fabs(currentPos.y - waypoint.pose.position.y) < cellThresh) and (math.fabs(currentPos.z-waypoint.pose.position.z) < cellThresh)) and not stuck:
            block = False
            checkArray=[]
            if(cellOccupied(waypoint.pose.position)):
                aFlag = True
                break
            elif((math.fabs(currentPos.x - waypoint.pose.position.x) < cellThresh)):
                tempY = currentPos.y
                tempYGoal=waypoint.pose.position.y
                for temp in range (int(round(tempY)), int(round(tempYGoal))):
                    tempPoint = Point()
                    tempPoint.x=currentPos.x
                    tempPoint.y=temp
                    checkArray.append(cellOccupied(tempPoint))
                for temp2 in checkArray:
                    if(temp2):
                        block = True
            elif((math.fabs(currentPos.y - waypoint.pose.position.y) < cellThresh)):
                tempX = currentPos.x
                tempXGoal=waypoint.pose.position.x
                for temp in range (int(round(tempX)),int(round(tempXGoal))):
                    tempPoint = Point()
                    tempPoint.x = temp
                    tempPoint.y = currentPos.y
                    checkArray.append(cellOccupied(tempPoint))
                for temp2 in checkArray:
                    if(temp2):
                        block = True
            if(block):
                aSTAR(currentPos,waypoint.pose)
                block=False
                fullDrive()
            navToPose(waypoint)
            stuck = True
	print("Way Point Reached!!!")




#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    global odom_list
    global pose
    global currentAngle
   # global map_list
    global goal_pose
    #global bot_pose
    #global geo_quat
    #global posx
    #global posy
    print currentAngle
  #  pose_Stamp = PoseStamped();
   # pose_Stamp.pose.position.x = pose.pose.position.x
    #pose_Stamp.pose.position.y = pose.pose.position.y
   # pose_Stamp.pose.orientation = pose.pose.orientation
    
   # transformer = tf.TransformerROS()
   # goal_list.waitForTransform('map', 'odom', rospy.Time(0), rospy.Duration(4.0))
   # bot_pose = transformer.transformPose('map', pose_Stamp)
    #Get transforms for frames
    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans,rot) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
    #start_angle = euler_from_quaternion(rot)
    
   
   # print "Starting Angle:"
    #print start_angle
    x0 = trans[0]	#Set origin
    y0 = trans[1]
    print "Origin"
    print x0
    print y0
    goal_pose = goal.pose
    print "goalx"
    print goal_pose.position.x
    print "goaly"
    print goal_pose.position.y
    
    x1 = goal_pose.position.x
    y1 = goal_pose.position.y
    xx = (x1 - x0)
    yy = (y1 - y0)
    adjust_angle = math.atan2(yy, xx)
    
    
    first_rotate_angle = adjust_angle - currentAngle
    print "Full Rotate"
    print first_rotate_angle
    
    rotate(first_rotate_angle)
    
    print "spin!"
    
    
    
   
    
    
    
    print "Goal"
    print x1
    print y1
   
    d = math.sqrt((math.pow(xx,2) + math.pow(yy,2)))	#Distance formula
    
    #print d
    
    driveStraight(.15, d)
    print "move!"
    quat = goal.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    
    final_angle_adjust = yaw - first_rotate_angle
    rotate(final_angle_adjust)
    currentAngle = yaw
    
    print "spin!"
    print currentAngle
    print "done"
    
    pass
    
#A *
def aSTAR(start,goal):
    global currentPoint
    global mapData
    global cells_met
    global cellThresh
    global doneFlag
    global cardinalDir
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
#searches through cells_met (the path) and determine waypoints off of what is not a front
def createWaypoints():
    global cells_met
    waypoints = []
    waypoint = PoseStamped()
    for i in range(0, len(cells_met.cells)): #goes from 0 to length of cells till goal
        if (cells_met.cells[i] != front):
            if (i == 0): #happens when obstacle is right in from 
                waypoint.header.frame_id = 'map'
                waypoint.pose.position.x = cells_met.cells[i].x
                waypoint.pose.position.y = cells_met.cells[i].y
                waypoint.pose.position.z = cells_met.cells[i].z
                waypoint.pose.orientation.z = 0 #until we think of better way but fine for now
                
                waypoints.append(waypoint) #adds waypoint to the list
                
            elif (i == len(cells_met.cells)):
                waypoint.header.frame_id = 'map'
                waypoint.pose.position.x = cells_met.cells[i].x
                waypoint.pose.position.y = cells_met.cells[i].y
                waypoint.pose.position.z = cells_met.cells[i].z
                waypoint.pose.orientation.z = 0 #until we think of better way but fine for now
                
                waypoints.append(waypoint) #adds waypoint to the list
                
            else:
                waypoint.header.frame_id = 'map'
                waypoint.pose.position.x = cells_met.cells[i-1].x
                waypoint.pose.position.y = cells_met.cells[i-1].y
                waypoint.pose.position.z = cells_met.cells[i-1].z
                waypoint.pose.orientation.z = 0 #until we think of better way but fine for now
            
                waypoints.append(waypoint) #adds waypoint to the list
    
    return waypoints
             
    
                
#read map data
def readWorldMap(data):
    # map listener
    global mapData, grid
    global width
    global height
    grid = data
    mapData = data.data
    width = data.info.width
    height = data.info.height
    
#read Goal position
def readGoal(msg):
    global initPos
    global goal
    global waypoints
    px = msg.pose.position.x
    py = msg.pose.position.y
    quat = msg.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xEnd
    global yEnd
    global thetaEnd
    #print initPos.pose.position.x
    #print initPos.pose.position.y
    xEnd = px
    yEnd = py
    thetaEnd = yaw * 180.0 / math.pi
    goal = Pose();
    goal.position.x = round(px)
    goal.position.y = round(py)
    goal.position.z = 0
    
    
    
    #create waypoints
    waypoints = []
    #begin aSTAR
    aSTAR(initPos,goal)
    fullDrive()
    #for i in range (0, len(waypoints)):
    #    navToPose(waypoints[i])
    print "You did it Dora"
    #print waypoints[(len(waypoints)-1)].pose.position.x   
    #print waypoints[(len(waypoints)-1)].pose.position.y
#get initail pose position from rviz
def startCallBack(data):
    global cardinalDir
    global threshHold
    global initPos
    px = data.pose.pose.position.x
    py = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xInit
    global yInit
    global thetaInit
    xInit = px
    yInit = py
    thetaInit = yaw * 180.0 / math.pi
    if((0 - thetaInit) < threshHold):
        cardinalDir = 4
    elif((90 - thetaInit) < threshHold):
        cardinalDir = 1
    elif((180 - thetaInit) < threshHold):
        cardinalDir = 2
    elif((360 - thetaInit) < threshHold):
        cardinalDir = 3
    else:
        print "Angle off or threshHold too low"
    
    initPos.header.frame_id = 'map'
    initPos.pose.position.x = px
    initPos.pose.position.y = py
    intiPos.pose.position.z = 0
    
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
    #set to currentPoint
    currentPoint = Point(); #might need to change to pose if neighbors dont work out when looking different directions
    currentPoint.x = x
    currentPoint.y = y
    currentPoint.z = 0
#publish or just get grid cells that have obstacle with x,y location
def publishObjectCells(grid):
    global pubGCell
    global unknownCell
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
    for i in range(1,len(occupiedCells.cells)):
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
    
    for i in range(1,height): #height should be set to hieght of grid

        b=b+1
        for j in range(1,width): #height should be set to hieght of grid
            b=b+1
            #print b # used for debugging
            if (grid[b] == 0):
                point1=Point()
                point1.x=j*.05 # edit for grid size
                point1.y=i*.05 # edit for grid size
                point1.z=0
                Cells.cells.append(point1)
    
    unknownCell.publish(Cells)
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
        print "PLEASE"
        print x
        print y
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
        print x
        print y
        print right.x
        print right.y
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

    cells_met.cells.append(right)
        
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
    
    cells_met.cells.append(left)
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
    
    
    cells_met.cells.append(front)
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

    cells_met.cells.append(back)    
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
#This function consumes linear and angular velocities
#and creates a Twist message.  This message is then published.
def publishTwist(lin_vel, ang_vel):
    global pub

    twist_msg = Twist();		#Create Twist Message

    twist_msg.linear.x = lin_vel	#Populate message with data
    twist_msg.angular.z = ang_vel
    pub.publish(twist_msg)		#Send Message
 
#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    global pub

    lin_vel = ((u1*wheel_rad)/2) + ((u2*wheel_rad)/2)			#Determines the linear velocity of base based on the wheels
    ang_vel = ((u1*wheel_rad)/(2*wheel_base)) - ((u2*wheel_rad)/(2*wheel_base))		#Determines the angular velocity of base on the wheels.

    #twist_msg = Twist();			#Creates two messages: 
    #stop_msg = Twist();

    #twist_msg.linear.x = lin_vel		#Populate messages with data.
    #twist_msg.angular.z = ang_vel
    #stop_msg.linear.x = 0
    #stop_msg.angular.z = 0
    

    #While the specified amount of time has not elapsed, send Twist messages.
    now = rospy.Time.now().secs
    while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
        publishTwist(lin_vel, ang_vel)
    
    publishTwist(0, 0)
    

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global odom_list
    global pose 	
    #print "Here" #was for debug
    x0 = pose.pose.position.x	#Set origin
    y0 = pose.pose.position.y

    #Loop until the distance between the attached frame and the origin is equal to the
    #distance specifyed 
    done = False
    while (not done and not rospy.is_shutdown()):
        x1 = pose.pose.position.x
        y1 = pose.pose.position.y
        xx = (x1 - x0)
        yy = (y1 - y0)
        d = math.sqrt((math.pow(xx,2) + math.pow(yy,2)))	#Distance formula
        
        if (d >= distance):
            done = True
            publishTwist(0, 0)
        else:
            publishTwist(speed, 0)

    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global odom_list
    global pose

    #This node was created using Coordinate system transforms and numpy arrays.
    #The goal is measured in the turtlebot's frame, transformed to the odom.frame 
    transformer = tf.TransformerROS()	
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],	#Create goal rotation
                            [math.sin(angle), math.cos(angle), 0],
                            [0,          0,          1]])

    #Get transforms for frames
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3,0:3]

    #Setup goal matrix
    goal_rot = numpy.dot(rotation, R_o_t)
    goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                    [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                    [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                    [0,             0,             0,             1]])

    #Continues creating and matching coordinate transforms.
    done = False
    while (not done and not rospy.is_shutdown()):
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        within_tolerance = abs((state - goal_o)) < .2
        if ( within_tolerance.all() ):
            spinWheels(0,0,0)
            done = True
        else:
            if (angle > 0):
                spinWheels(1.3,-1.3,.1)
            else:
                spinWheels(-1.3,1.3,.1)
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
    global initPos
    global currentAngle
    #global goal_pose
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
    #goal = Point();
    #start = PoseStamped();
    
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
   
