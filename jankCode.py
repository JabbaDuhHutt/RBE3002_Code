#!/usr/bin/env python


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

def statusReader(msg):
    global move_status

    #gets status array
    move_status = msg.status_list
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

def initialSpin():
    global currentPoint
    global pubMove
    global UnknownSpace

    readCurrentPos()
    
    #move1 = PoseStamped()
    #move1.header.frame_id = 'map'
    #move1.pose.position.x = currentPoint.x + .1
    #move1.pose.position.y = currentPoint.y + .1
    #move1.pose.orientation.z = 0
    
    #pubMove.publish(move1)
    
    move2 = PoseStamped()
    move2.header.frame_id = 'map'
    move2.pose.position.x = currentPoint.x
    move2.pose.position.y = currentPoint.y
    move2.pose.orientation.z = 6.28

    pubMove.publish(move2)

    navToFrontiers()
def navToFrontiers():
    global move_status
    global height
    global width

    done = False
    i = 0
    j = 0
    while not done:
        movement1 = PoseStamped()
        movement1.header.frame_id = 'map'
        movement1.pose.position.x = width * 0.05
        movement1.pose.position.y = height * 0.05
        movement1.pose.orientation.z = 0

        pubMove.publish(movement1)

        wait = 0
        #cancels after wait has been too long for succeed
        while (not (move_status[i].status == 3)  or not done):
            #set wait to time out nav after certain time
            if(wait>=20000):
                print "Nav failed"
                cancelMove.publish(move_status[i].goal_id)
                done = True
                i += 1
                break
            else:

                #chill
                print "Waiting for nav to finish"
                wait += 1
            
                #increments so that proper array is looked at when nav
                i += 1
    done = False
    while not done:
        movement2 = PoseStamped()
        movement2.header.frame_id = 'map'
        movement2.pose.position.x = -(width * 0.05) #either negative or 0 cant decide
        movement2.pose.position.y = height * 0.05
        movement2.pose.orientation.z = 0

        pubMove.publish(movement2)

        wait = 0
        #cancels after wait has been too long for succeed
        while (not (move_status[i].status == 3)  or not done): #maybe i or maybe not
            #set wait to time out nav after certain time
            if(wait>=20000):
                print "Nav failed"
                cancelMove.publish(move_status[i].goal_id)
                done = True
                i += 1
                break
            else:
                
                #chill
                print "Waiting for nav to finish"
                wait += 1
                #increments so that proper array is looked at when nav
                i += 1
    done = False
    while not done:
        movement3 = PoseStamped()
        movement3.header.frame_id = 'map'
        movement3.pose.position.x = width * 0.05 #either negative or 0 cant decide
        movement3.pose.position.y = -(height * 0.05)
        movement3.pose.orientation.z = 0

        pubMove.publish(movement3)

        wait = 0
        #cancels after wait has been too long for succeed
        while (not (move_status[i].status == 3) or not done): #maybe i or maybe not
            #set wait to time out nav after certain time
            if(wait>=20000):
                print "Nav failed"
                cancelMove.publish(move_status[i].goal_id)
                done = True
                i += 1
                break
            else:
                
                #chill
                print "Waiting for nav to finish"
                wait += 1
                #increments so that proper array is looked at when nav
                i += 1
    done = False
    while not done:
        movement4 = PoseStamped()
        movement4.header.frame_id = 'map'
        movement4.pose.position.x = -(width * 0.05) #either negative or 0 cant decide
        movement4.pose.position.y = -(height * 0.05)
        movement4.pose.orientation.z = 0

        pubMove.publish(movement4)

        wait = 0
        #cancels after wait has been too long for succeed
        while (not (move_status[i].status == 3)  or not done): #maybe i or maybe not
           #set wait to time out nav after certain time
            if(wait>=20000):
                print "Nav failed"
                cancelMove.publish(move_status[i].goal_id)
                done = True
                i += 1
                break
            else:
                
                #chill
                print "Waiting for nav to finish"
                wait += 1
                #increments so that proper array is looked at when nav
                i += 1
    print "DONE"
    


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

    #q = [odomW.x, odomW.y, odomW.z, odomW.w]

    roll, pitch, yaw = euler_from_quaternion(q)

    #convert yaw to degrees

    global currentAngle

    #print currentTheta

    currentAngle = yaw #to correct for driving issues

    currentTheta = math.degrees(yaw)

    #if(math.fabs(0 - currentTheta) < threshHold): #might need to do negative angle and positive (also might want <=)

    #    cardinalDir = 4

    #elif(math.fabs(90 - currentTheta) < threshHold):

     #   cardinalDir = 1

    #elif(math.fabs(180 - currentTheta) < threshHold):

     #   cardinalDir = 2

   # elif(math.fabs(270 - currentTheta) < threshHold):

       # cardinalDir = 3

    #else:

       # print "Angle off or threshHold too low"

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
    frontLeft = Point();
    frontRight = Point();
    left = Point();
    right = Point(); 
    backLeft = Point();
    backRight = Point();
    back = Point(); #might need to change depending on cells

    

    initPos = PoseStamped();

    

    rospy.init_node('lab3')

    

    #subscribers

    worldMapSub = rospy.Subscriber('/map', OccupancyGrid, readWorldMap)

    #markerSub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, readGoal)

    #odomSub = rospy.Subscriber('/odom', Odometry, readOdom)

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

    #odom_tf = tf.TransformBroadcaster()

    

    #odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"map","base_footprint")

    

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