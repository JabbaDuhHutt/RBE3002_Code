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

#A *
def aSTAR(start,goal):
    global currentPoint
    global mapData
    
    startPos = start.pose.position
    goalPos = goal.position

    publishObjectCells(mapData)
    while(not doneFlag and not rospy.is_shutdown()):
        #lowest
        #direction
        costFront=9001
        costLeft=9001
        costRight=9001
        costBack=9001 #derp I capitalized C by accident##############################################################################################################
        hugh = heuristic(startPos,goalPos)
        readCurrentPos()
        mrRogers(currentPoint)
        if(not cellOccupied(front)):#im adding not cause I think I did logic wrong
            costFront=distanceFormula(currentPoint,front)#seems to fix it but still i think there is a problem with if it is occupied nvm(its capitalization)
            costFront+=heuristic(front,goalPos)
        if(not cellOccupied(back)):
            costBack=distanceFormula(currentPoint,back)
            costBack+=heuristic(back,goalPos)
        if(not cellOccupied(left)):
            costLeft=distanceFormula(currentPoint,left)
            costLeft+=heuristic(left,goalPos)
        if(not cellOccupied(right)):
            costRight=distanceFormula(currentPoint,right)
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
            goFront()
        elif(direction=="right"):
            goRight()
        elif(direction=="back"):
            goBack()
        elif(direction=="left"):
            goLeft()
    print "DONE"
            
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
    px = msg.pose.position.x
    py = msg.pose.position.y
    quat = msg.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    global xEnd
    global yEnd
    global thetaEnd
    xEnd = px
    yEnd = py
    thetaEnd = yaw * 180.0 / math.pi
    goal = Pose();
    goal.position.x = px
    goal.position.y = py
    goal.position.z = 0
    
    #begin aSTAR
    aSTAR(initPos,goal)
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
        cardinalDir = 'D'
    elif((90 - thetaInit) < threshHold):
        cardinalDir = 'A'
    elif((180 - thetaInit) < threshHold):
        cardinalDir = 'B'
    elif((360 - thetaInit) < threshHold):
        cardinalDir = 'C'
    else:
        print "Angle off or threshHold too low"
    
    initPos.header.frame_id = 'start'
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
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))###############################################################################I mad changes here
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
    global currentTheta
    currentTheta = math.degrees(yaw)
    if(math.fabs(0 - currentTheta) < threshHold): #might need to do negative angle and positive (also might want <=)
        cardinalDir = 'D'
    elif(math.fabs(90 - currentTheta) < threshHold):
        cardinalDir = 'A'
    elif(math.fabs(180 - currentTheta) < threshHold):
        cardinalDir = 'B'
    elif(math.fabs(270 - currentTheta) < threshHold):
        cardinalDir = 'C'
    else:
        print "Angle off or threshHold too low"
    #set to currentPoint
    currentPoint = Point(); #might need to change to pose if neighbors dont work out when looking different directions
    currentPoint.x = x
    currentPoint.y = y
    currentPoint.z = 0
#publish or just get grid cells that have obstacle with x,y location
def publishObjectCells(grid):
    global height
    global width
    global occupiedCells
    k = 0
    occupiedCells = GridCells()
    occupiedCells.header.frame_id = 'map'
    occupiedCells.cell_width = 0.3 #change based off grid size
    occupiedCells.cell_height = 0.3 #change based off grid size
    
    for i in range(1,height): #height should be set to hieght of grid
        k=k+1
        for j in range(1,width): #height should be set to hieght of grid
            k=k+1
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=j*.3 # edit for grid size
                point.y=i*.3 # edit for grid size
                point.z=0
                occupiedCells.cells.append(point)
    
    
#adjusts global neighbors to current neighbors
def mrRogers(current):
    #global pose #dont think this needs to be here
    global front
    global left
    global right
    global back
    global unit_cell
    global cardinalDir
    global height
    global width
    
    print "Will you be my neighbor"
    if(cardinalDir == 'A'):
        front.x = current.x
        front.y = current.y + unit_cell
        front.z = 0
        print "front neighbor found"
        left.x = current.x - unit_cell #if point gets negative then off map do something to deal with this
        left.y = current.y
        left.z = 0
        print "left neighbor found"
        back.x = current.x
        back.y = current.y - unit_cell
        back.z = 0
        print "back neighbor found"
        right.x = current.x + unit_cell
        right.y = current.y
        right.z = 0
        print "right neighbor found"
    elif(cardinalDir == 'B'):
        front.x = current.x - unit_cell
        front.y = current.y
        front.z = 0
        print "front neighbor found"
        left.x = current.x
        left.y = current.y - unit_cell
        left.z = 0
        print "left neighbor found"
        back.x = current.x + unit_cell
        back.y = current.y 
        back.z = 0
        print "back neighbor found"
        right.x = current.x
        right.y = current.y + unit_cell
        right.z = 0
        print "right neighbor found"
    elif(cardinalDir == 'C'):
        front.x = current.x
        front.y = current.y - unit_cell
        front.z = 0
        print "front neighbor found"
        left.x = current.x + unit_cell
        left.y = current.y
        left.z = 0
        print "left neighbor found"
        back.x = current.x
        back.y = current.y + unit_cell
        back.z = 0
        print "back neighbor found"
        right.x = current.x - unit_cell
        right.y = current.y
        right.z = 0
        print "right neighbor found"
    else:
        front.x = current.x + unit_cell
        front.y = current.y
        front.z = 0
        print "front neighbor found"
        left.x = current.x
        left.y = current.y + unit_cell
        left.z = 0
        print "left neighbor found"
        back.x = current.x - unit_cell
        back.y = current.y 
        back.z = 0
        print "back neighbor found"
        right.x = current.x
        right.y = current.y - unit_cell
        right.z = 0
        print "right neighbor found"
#takes cell of point() and returns whether the cell is occupied or not
def cellOccupied(cell):
    global occupiedCells
    global cellThresh
    #for each occupiedCell compare the point to point that was passed in
    for occupied in occupiedCells.cells:
        if((math.fabs(occupied.x - cell.x) < cellThresh) and (math.fabs(occupied.y - cell.y) < cellThresh) and (math.fabs(occupied.z - cell.z) < cellThresh)): #break up for debug if not equating 
            return True
        else:
            return False
#currently heuristic is just straightline between point(start) and goal 
def heuristic(start,goal):
    global doneFlag

    
    distance = distanceFormula(start, goal)
    if(distance==0):
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
#turns right and goes straight 1 cell
def goRight():
    global unit_cell
    rotate(-1.571)
    driveStraight(.2,unit_cell)
    
#turns left and goes straight 1 cell
def goLeft():
    global unit_cell
    rotate(1.571)
    driveStraight(.2,unit_cell)
    
#goes straight 1 cell
def goFront():
    global unit_cell
    driveStraight(.2,unit_cell)
    
#turns 180 and goes straight 1 cell
def goBack():
    global unit_cell
    rotate(3.14)
    driveStraight(.2,unit_cell)
    
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
    x0 = pose.position.x	#Set origin
    y0 = pose.position.y

    #Loop until the distance between the attached frame and the origin is equal to the
    #distance specifyed 
    done = False
    while (not done and not rospy.is_shutdown()):
        x1 = pose.position.x
        y1 = pose.position.y
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
                spinWheels(1,-1,.1)
            else:
                spinWheels(-1,1,.1)
#Odometry Callback function
#def readOdom(msg):
#    global pose
#    global odom_tf

#    pose = msg.pose
#    geo_quat = pose.pose.orientation

#    odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0), (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w), rospy.Time.now(), "base_footprint","odom")
    
def run():
    global worldMap
    #global start
    global target
    global front
    global left
    global right
    global back
    #global odom_tf
    global odom_list
    global pose
    global unit_cell
    global currentTheta #theta of current robot to map
    global currentPoint #might need to change to pose if neighbors dont work out
    global cardinalDir #direction robot is facing in respect to global map (A is +y , B is -x, C is -y, D is +x)
    global occupiedCell #list of occupied cells
    global initPos
    #global goal
    global doneFlag
    global threshHold
<<<<<<< HEAD
    global pub
=======
>>>>>>> origin/master
    global cellThresh
    unit_cell = .30 #m
    AMap = 0
    worldMap = 0
    path = 0
    
    cellThresh = .05 #m
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
    #odomSub = rospy.Subscriber('/odom', Odometry, readOdom)
    sub = rospy.Subscriber('/initialPose', PoseWithCovarianceStamped, startCallBack)
    #publishers
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
    cellPub = rospy.Publisher('/cell_path', GridCells, queue_size = 1)
    pathPub = rospy.Publisher('/path_path', Path, queue_size = 1)
    
    #listener/broadcaster might not be needed but here
    odom_list = tf.TransformListener()
    #odom_tf = tf.TransformBroadcaster()
    
    #odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")
    
    sleeper = rospy.Duration(1)
    rospy.sleep(sleeper)
    
    target = 0
    start = 0
    end = 0

    while not rospy.is_shutdown():
        print("starting")
        print("waiting")
        rospy.loginfo("Waiting...")
        rospy.spin() 
        #rospy.is_shutdown()


    # resolution and offset of the map

    # create a new instance of the map

    # generate a path to the start and end goals

    # for each node in the path, process the nodes to generate GridCells and Path messages
  
    # transform coordinates for map resolution and offset

    # continue making messages

    # do not stop publishing
    
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
   
