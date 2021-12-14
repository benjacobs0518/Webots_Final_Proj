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
class AStarGraph(object):
#Define a class board like grid with two barriers
 
    def __init__(self, map, row, col):
        self.barriers = []
        for x in range(row):
            for y in range(col):
                if(map[x][y] == 1):
                    self.barriers.append((x,y)) 
		
    def checker(self):
        print(self.barriers)
        #print(len(self.barriers))
            	
	
    def heuristic(self, start, goal):
        #Use Chebyshev distance heuristic if we can move one square either
        #adjacent or diagonal
        D = 1
        D2 = 1
        dx = abs(start[0] - goal[0])
        dy = abs(start[1] - goal[1])
        return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
 
    def get_vertex_neighbours(self, pos):
        n = []
        #Moves allow link a chess king
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
            x2 = pos[0] + dx
            y2 = pos[1] + dy
            if x2 < 0 or x2 > 360 or y2 < 0 or y2 > 360:
                continue
            n.append((x2, y2))
        return n
 
    def move_cost(self, a, b):
        for barrier in self.barriers:
            if b == barrier:
                return 10000000 #Extremely high cost to enter barrier squares
        return 1 #Normal movement cost
 
def AStarSearch(start, end, graph):
   
    #print(graph.barriers)
    G = {} #Actual movement cost to each position from the start position
    F = {} #Estimated movement cost of start to end going via this position
    
    #Initialize starting values
    G[start] = 0
    F[start] = graph.heuristic(start, end)
    
    closedVertices = set()
    openVertices = set([start])
    cameFrom = {}
    
    while len(openVertices) > 0:
        #Get the vertex in the open list with the lowest F score
        current = None
        currentFscore = None
        for pos in openVertices:
            if current is None or F[pos] < currentFscore:
                currentFscore = F[pos]
                current = pos
        
        #Check if we have reached the goal
        if current == end:
            #Retrace our route backward
            path = [current]
            while current in cameFrom:
                current = cameFrom[current]
                path.append(current)
            path.reverse()
            return path, F[end] #Done!
        
        #Mark the current vertex as closed
        openVertices.remove(current)
        closedVertices.add(current)
        
      
        #Update scores for vertices near the current position
        for neighbour in graph.get_vertex_neighbours(current):
           
            if neighbour in closedVertices:
                continue #We have already processed this node exhaustively
            candidateG = G[current] + graph.move_cost(current, neighbour)
            
            if neighbour not in openVertices:
                openVertices.add(neighbour) #Discovered a new vertex
            elif candidateG >= G[neighbour]:
                continue #This G score is worse than previously found
         
            #Adopt this G score
            cameFrom[neighbour] = current
            G[neighbour] = candidateG
            H = graph.heuristic(neighbour, end)
            F[neighbour] = G[neighbour] + H
    
    
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
#AXLE_LENGTH = 
#MAX_SPEED_MS = 

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
    robot_parts[i].setVelocity(MAX_SPEED)
    
    
def set_speed(left, right):
    for i in range(4):
        robot_parts[i].setVelocity(left)
        robot_parts[i+4].setVelocity(right)

waypoints = []
state = 0 
d = 0
b = True


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
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

        # Normalize wheelspeed
        # (Keep the wheel speeds a bit less than the actual platform MAX_SPEED to minimize jerk)
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
                # print("WAYPOINT",state)
            if(not(state < len(waypoints))):
                b = False
                vL = 0
                vR = 0
    else:
        vL = 0
        vR = 0
    # Odometry code. Don't change vL or vR speeds after this line.
    # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    # print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta))

    # Actuator commands
    set_speed(vL, vR)
    #robot_parts[MOTOR_LEFT].setVelocity(vL)
    #robot_parts[MOTOR_RIGHT].setVelocity(vR)