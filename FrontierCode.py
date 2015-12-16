#!/usr/bin/env python

class Frontier:
    
    def __init__(self,centroid,size,min_distance):
        self.centroid = centroid
        self.size= size
        self.min_distance = min_distance
        self.middle = Point()

import rospy

#import Frontier

from std_msgs.msg import String

from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry

from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion

from actionlib_msgs.msg import GoalStatusArray, GoalID

from tf.transformations import euler_from_quaternion

import tf

import numpy

import math 

import Queue

#debug github
#intializes search by taking in current position
def searchFrontiers():
    global currentPoint
    #global cardinalDir
    global front
    global right
    global left
    global back
    global visitedList
    global frontier_flag
    global frontier_list

    #cardinalDir = 1

    frontier_list = []

#move current point to local
    readCurrentPos()
    cx = currentPoint.x
    cy = currentPoint.y
#create local point for start
    startPoint = Point();
    startPoint.x = cx
    startPoint.y = cy

    visitedList = []
    frontier_flag = []

    bfs = Queue.Queue()
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
            t = len(visitedList)
            if(((not cellOccupied(x)) and (not unknownSpace(x))) and  not visited(x, t)):
                visitedList.append(x) #marks cell as visited
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
        if(unknownSpace(x)):
            return True

    return False
#builds new frontier based on given cell
def buildNewFrontier(cell):
    global currentPoint
    global front
    global frontLeft
    global left
    global frontRight
    global right
    global backLeft
    global back
    global backRight
    global frontier_flag

    centroidPoint= Point()
    centroidPoint.x=0
    centroidPoint.y=0
    centroidPoint.z=0
    readCurrentPos()
    output = Frontier(centroidPoint,1,9001)

    bfs2=Queue.Queue()
    bfs2.put(initial_cell)
    


    #take out reference

    while not bfs2.empty():
        idx = bfs2.get()

        EightNeighbors(idx)
        fx = front
        flx = frontLeft
        lx = left
        frx = frontRight
        rx = right
        blx = backLeft
        bx = back
        brx = backRight
        nbr = []
        nbr.append(fx)
        nbr.append(flx)
        nbr.append(lx)
        nbr.append(frx)
        nbr.append(rx)
        nbr.append(blx)
        nbr.append(bx)
        nbr.append(brx)

        for j in nbr:
            if(isNewFrontierCell(j)):
                frontier_flag.append(j)
                output.size = output.size + 1
                output.centroid.x = output.centroid.x + j.x
                output.centroid.y = output.centroid.y + j.y

            distance = distanceFormula(currentPoint,j)
            if(distance < output.min_distance):
                output.min_distance = distance
                output.middle.x = j.x
                output.middle.y = j.y

            bfs2.put(j)

    output.centroid.x = output.centroid.x/output.size
    output.centroid.y = output.centroid.y/output.size
    return output


def pointConversionToGmapping(pose):
    global origin
    
    newPose = Pose()
    newPose.position.x = pose.position.x + origin.position.x
    newPose.position.y = pose.position.y + origin.position.y
    newPose.orientation.z = pose.orientation.z + origin.orientation.z #i think this is right + over -

    return newPose
     
def pointConversionToReferrence(pose):
    global origin
    
    newPose = Pose()
    newPose.position.x = pose.position.x - origin.position.x
    newPose.position.y = pose.position.y - origin.position.y
    newPose.orientation.z = pose.orientation.z - origin.orientation.z #i think this is right + over -

    return newPose

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

    publishObjectCells(mapData)

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
def statusReader(msg):
    global move_status

    #gets status array
    move_status = msg.status_list

def initialSpin():
    global currentPoint
    global pubMove
    global UnknownSpace

    readCurrentPos()

    move1 = PoseStamped()
    move1.header.frame_id = 'map'
    move1.pose.position.x = currentPoint.x
    move1.pose.position.y = currentPoint.y
    move1.pose.orientation.z = 3.4

    pubMove.publish(move1)
    #sleep for 5 sec for gmapping to catch up
    rospy.sleep(5.)
    #start navigating to frontiers
    navToFrontiers()
    
    #keeps exploring till all frontiers reached, but might not finish if we arent careful (doesnt stop exploring)
    while( not (len(UnknownSpace.cells) <=1)):
        #sleep for 5 sec for gmapping to catch up
        rospy.sleep(5.)
        navToFrontiers()
    print "We did it!!"
    print "All frontiers explored"

#navigates to frontiers
def navToFrontiers():
    global move_status
    global cancelMove

    i = 0
    frontiers = searchFrontiers()

    for frontier in frontiers:
        #pose of given centroid
        frontierConvert = Pose()
        frontierConvert.position.x = frontier.centroid.x
        frontierConvert.position.y = frontier.centroid.y

        #convert to gmapping (if occupancy grid is how we hope)
        goal = pointConversionToGmapping(frontierConvert)


        tempPose = PoseStamped()
        tempPose.header.frame_id = 'map'
        tempPose.pose.position.x = goal.position.x
        tempPose.pose.position.y = goal.position.y
        #publish posestamp messave to move stack
        pubMove.publish(tempPose)

    wait = 0
    #cancels after wait has been too long for succeed
    while( (not i == len(move_status))):
        while (not (move_status[i].status == 3)):
            #chill
            print "Waiting for nav to finish"
            wait += 1
            #set wait to time out nav after certain time
            if(wait>=20000):
                print "Nav failed"
                cancelMove.publish(move_status[i].goal_id)
                i += 1
                break
            #increments so that proper array is looked at when nav
            i += 1
    print "Navigation should be done"
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

 
#eight neighbors
def EightNeighbors(current):
    global front
    global frontLeft
    global left
    global frontRight
    global right
    global backLeft
    global back
    global backRight
    
    global unit_cell

    global cardinalDir

    global height

    global width
    
    x = current.x
    y = current.y
    
    front.x = x
    front.y = y + unit_cell
    front.z = 0
    frontLeft.x = x - unit_cell
    frontLeft.y = y + unit_cell
    frontLeft.z = 0    
    frontRight.x = x + unit_cell
    frontRight.y = y + unit_cell
    frontRight.z = 0
    back.x = x
    back.y = y - unit_cell
    back.z = 0
    backLeft.x = x - unit_cell
    backLeft.y = y - unit_cell
    backLeft.z = 0
    backRight.x = x + unit_cell
    backRight.y = y - unit_cell
    backRight.z = 0
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
    global visitedList


    #check visited for "cell"
    #if cell has been visited return true
    if(length <= 0):
        for occupied in visitedList:

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
    
    #pubGCell.publish(occupiedCells)

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

        if(unknown not in UnknownSpace.cells):
            UnknownSpace.cells.append(unknown)
            
    #unknownCell.publish(Cells)
    
def distanceFormula(start1,goal1):

    x0 = start1.x

    y0 = start1.y

    

    x1 = goal1.x

    y1 = goal1.y

    

    xx = x1-x0

    yy = y1-y0

    d = math.sqrt((math.pow(xx,2) + math.pow(yy,2)))

    return d

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

    #global target

    global front
    global frontLeft
    global left
    global frontRight
    global right
    global backRight
    global back
    global backLeft

    global move_status

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

    global pubMove

    global cancelMove

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
    

    move_status = []

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

    #markerSub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, readGoal)

    odomSub = rospy.Subscriber('/odom', Odometry, readOdom)

    subMove = rospy.Subscriber('/move_base/status', GoalStatusArray, statusReader)

    

    #publishers

    #pubGCell = rospy.Publisher('/grid_check', GridCells, queue_size=1)

    cancelMove = rospy.Publisher('/move_base/cancel', GoalID, queue_size = 1)

    pubMove = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)

    #cellPub = rospy.Publisher('/cell_path', GridCells, queue_size = 100)

    #pathPub = rospy.Publisher('/path_path', Path, queue_size = 1)

    #unknownCell = rospy.Publisher('/grid_unknown', GridCells, queue_size=1)

    

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

        initialSpin()
        #odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"map","base_footprint")

        #publishObjectCells(mapData)
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
        