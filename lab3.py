#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy
import math 

#Kobuki Dimensions
wheel_rad  = 0.035  #m
wheel_base = 0.23 #m

#read map data
def readWorldMap(data):
    
#read Goal position
def readGoal(msg):
    
#get initail pose position from rviz
def startCallBack(data):
    
#publish or just get grid cells that have obstacle with x,y location
def publishObjectCells(grid):
    
#adjusts global neighbors to current neighbors
def mrRogers(current):
    print "Will you be my neighbor"
    
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
                spinWheels(1,-1,.1)
            else:
                spinWheels(-1,1,.1)
#Odometry Callback function
def readOdom(msg):
    global pose
    global odom_tf

    pose = msg.pose
    geo_quat = pose.pose.orientation

    odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0), (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w), rospy.Time.now(), "base_footprint","odom")
    
def run():
    global worldMap
    global target
    global front
    global left
    global right
    global back
    global odom_tf
    global odom_list
    global pose
    global unit_cell

    unit_cell = .30 #m
    AMap = 0
    worldMap = 0
    path = 0

    rospy.init_node('lab3')
    #subscribers
    worldMapSub = rospy.Subscriber('/map', OccupancyGrid, readWorldMap)
    markerSub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, readGoal)
    odomSub = rospy.Subscriber('/odom', Odometry, readOdom)
    sub = rospy.Subscriber("/initialPose", PoseWithCovarianceStamped, startCallBack)
    #publishers
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist,queue_size=1)
	cellPub = rospy.Publisher('/cell_path', GridCells)
    pathPub = rospy.Publisher('/path_path', Path)
    
    #listener/broadcaster might not be needed but here
    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()
    
    odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")
    
    sleeper = rospy.Duration(1)
    rospy.sleep(sleeper)

    target = 0
    start = 0
    end = 0



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
   
