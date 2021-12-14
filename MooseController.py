"""MooseController controller."""

# You may need to import some classes of the controller module. Ex:
 # from controller import Robot, Motor, DistanceSensor
from __future__ import print_function
from controller import Robot, Motor, DistanceSensor
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space
from queue import PriorityQueue

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
 # motor = robot.getDevice('motorname')
 # ds = robot.getDevice('dsname')
 # ds.enable(timestep)

TARGET_POINTS_SIZE = 1
DISTANCE_TOLERANCE = 1.5
MAX_SPEED = 7.0
TURN_COEFFICIENT = 4
N_PARTS = 8
AXLE_LENGTH = 0.8
MAX_SPEED_MS = 2.236

names = ["left motor 1",  "left motor 2",  "left motor 3",  "left motor 4",
                          "right motor 1", "right motor 2", "right motor 3", "right motor 4"];

timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

robot_parts = []

for i in range(8):
    robot_parts.append(robot.getDevice(names[i]));
    robot_parts[i].setPosition(float('inf'));
    robot_parts[i].setVelocity(0)
    #robot_parts[i].setVelocity(MAX_SPEED)
    
    
def set_speed(left, right):
    for i in range(4):
        robot_parts[i].setVelocity(left)
        robot_parts[i+4].setVelocity(right)

map = np.zeros((1000,500)) 
waypoints = []
state = 0 
d = 0
b = True

for i in range(100):
      robot.step(timestep)
      
      
#print(timestep*100)   #ms  
#set_speed(0,0)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

   
    # pass
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    if(b):
        rho = math.sqrt((pose_x-waypoints[state][0])**2 + (pose_y-waypoints[state][1])**2)
        alpha = -(math.atan2(waypoints[state][1]-pose_y,waypoints[state][0]-pose_x) + pose_theta)
        

        #STEP 2: Controller
    dX = 1.0*rho
    dTheta = 1.7*alpha

        #STEP 3: Compute wheelspeeds
    vL = (dX-dTheta*AXLE_LENGTH/2.0)
    vR = (dX+dTheta*AXLE_LENGTH/2.0)

        # ##Normalize wheelspeed
        # ##(Keep the wheel speeds a bit less than the actual platform MAX_SPEED to minimize jerk)
    vL = vL/MAX_SPEED_MS*MAX_SPEED
    vR = vR/MAX_SPEED_MS*MAX_SPEED
        
    proportion = vL/vR
    
    if(vL > MAX_SPEED or vR > MAX_SPEED):
        if(vL > MAX_SPEED and vL > vR):
            vL = MAX_SPEED
            vR = 1.0/proportion*vL
        else:
            vR = MAX_SPEED
            vL = proportion*vR
    if(b):
        if(abs(pose_x-waypoints[state][0]) < 0.5 and abs(pose_y-waypoints[state][1]) < 0.5):
            state += 1
                # ##print("WAYPOINT",state)
            if(not(state < len(waypoints))):
                b = False
                vL = 0
                vR = 0
    else:
        vL = 0
        vR = 0
    # ##Odometry code. Don't change vL or vR speeds after this line.
    # ##We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    # ##print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta))

    # ##Actuator commands
    set_speed(vL, vR)
    #robot_parts[MOTOR_LEFT].setVelocity(vL)
    #robot_parts[MOTOR_RIGHT].setVelocity(vR)