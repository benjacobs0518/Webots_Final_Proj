"""lab5 controller."""
from __future__ import print_function
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space
from queue import PriorityQueue

#MOST A* SOURCE CODE FROM: https://rosettacode.org/wiki/A*_search_algorithm#Python 
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
    
    #raise RuntimeError("A* failed to find a solution")

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 2.75 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)


##### vvv [Begin] Do Not Modify vvv #####

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# The Tiago robot has a couple more sensors than the e-Puck
# Some of them are mentioned below. We will use its LiDAR for Lab 5

# range = robot.getDevice('range-finder')
# range.enable(timestep)
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# The display is used to display the map. We are using 360x360 pixels to
# map the 12x12m2 apartment
display = robot.getDevice("display")

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

map = None
##### ^^^ [End] Do Not Modify ^^^ #####



##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
mode = 'manual' # Part 1.1: manual mode
# mode = 'planner'
# mode = 'autonomous'




###################
#
# Planner
#
###################
if mode == 'planner':
    # Part 2.3: Provide start and end in world coordinate frame and convert it to map's frame
    start_w = None # (Pose_X, Pose_Z) in meters
    end_w = None # (Pose_X, Pose_Z) in meters

    # Convert the start_w and end_w from the webots coordinate frame into the map frame
    start = None # (x, y) in 360x360 map
    end = None # (x, y) in 360x360 map

    # Part 2.3: Implement A* or Dijkstra's Algorithm to find a path
    def path_planner(map, start, end):
        '''
        :param map: A 2D numpy array of size 360x360 representing the world's cspace with 0 as free space and 1 as obstacle
        :param start: A tuple of indices representing the start cell in the map
        :param end: A tuple of indices representing the end cell in the map
        :return: A list of tuples as a path from the given start to the given end in the given maze
        '''
        pass

    # Part 2.1: Load map (map.npy) from disk and visualize it
    map1 = np.load("map.npy")
    plt.imshow(np.rot90(map1,3)) 
    plt.show()
    
    # Part 2.2: Compute an approximation of the “configuration space”
    map_temp = map1.copy()
    # plt.imshow(np.rot90(map_temp,3)) 
    # plt.show()
    k = 9
    for i in range(len(map1)):
        for j in range(len(map1[i])):
            if(map_temp[i,j] == 1):
                for d in range(k):
                    for e in range(k):
                        if(not(i-(k-1)/2+d < 0 or i-(k-1)/2+d > 359 or j-(k-1)/2+e < 0 or j-(k-1)/2+e > 359)):
                            map1[int(i-(k-1)/2+d),int(j-(k-1)/2+e)] = 1
    # plt.imshow(np.rot90(map1,3)) 
    # plt.show()
    # print(len(map1))
    # print(map1.size)

    # Part 2.3 continuation: Call path_planner
    graph = AStarGraph(map1, 360, 360)
    # graph.checker()
    result, cost = AStarSearch((int(360-int(8.05674*30)),int(4.46793*30)), (int(7/12*360),int(10/12*360)), graph)
    # print("route", result)  
    for h in result:
        map1[h[0],h[1]] = 2
    
    # plt.imshow(np.rot90(map1,3))
    # plt.show()  
    
    # Part 2.4: Turn paths into waypoints and save on disk as path.npy and visualize it
    waypoints = []
    
    # NOTE: if a* function worked, this would be used for part 2.4
    
    for m in result:
        # print((m[1]/30.0,(-m[0]+360)/30.0))
        waypoints.append((m[1]/30.0,(-m[0]+360)/30.0))
    np.save("path.npy",waypoints)

######################
#
# Map Initialization
#
######################

# Part 1.2: Map Initialization

# Initialize your map data structure here as a 2D floating point array
map = np.zeros((500,1000)) # Replace None by a numpy 2D floating point array
map_disaster = np.zeros((500,1000))
waypoints = []

if mode == 'autonomous':
    # Part 3.1: Load path from disk and visualize it
    # NOTE: if A* worked, the following code would be used
    waypoints = np.load("path.npy") # Replace with code to load your path
    # print(waypoints)
    # NOTE: Manual path from starting position to around 10.0,7.0 in meters
    # waypoints = [[5.5,6],[7,5.1],[9,5.1]]
    
state = 0 # use this to iterate through your path

# m = 0
end_position = []
d = 0
b = True
while robot.step(timestep) != -1 and mode != 'planner':

    ###################
    #
    # Mapping
    #
    ###################

    ################ v [Begin] Do not modify v ##################
    # Ground truth pose
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y
    
        ################ ^ [End] Do not modify ^ ##################
        
        scale = 17
        # print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))
        if rho < LIDAR_SENSOR_MAX_RANGE:
            # Part 1.3: visualize map gray values.
            if(map[500-int((wy+13.5)*scale),int((wx+26.0)*scale)] < 1):
                temp = camera.getRecognitionNumberOfObjects()
                map[500-int((wy+13.5)*scale),int((wx+26.0)*scale)] += 0.005
                if(camera.getRecognitionNumberOfObjects() != 0):
                    end_position.append(500-int((wy+13.5)*scale))
                    end_position.append(int((wx+26.0)*scale))
                    print(end_position[0])
                    print(end_position[1])
            
            #print(camera.getRecognitionNumberOfObjects())
            
            
            # You will eventually REPLACE the following 2 lines with a more robust version of the map
            # with a grayscale drawing containing more levels than just 0 and 1.
            g = int(map[500-int((wy+13.5)*scale),int((wx+26.0)*scale)]*255)
            color = g*256**2+g*256+g
            # display.setColor(color)
            # display.setColor(int(0x000000))
            display.setColor(int(0xFFFFFF))
            # if(objects < temp):
                # display.setColor(int(0xFF0000))
                # print(500-int((wy+13.5)*scale),int((wx+26.0)*scale))
            # objects = temp
            display.drawPixel(500-int((wy+13.5)*scale),int((wx+26.0)*scale))
            
            # display.setColor(0xFFFFFF)
            # display.drawPixel(360-int(wy*scale),-int(wx*scale))
            # print(500-int((wy+13.5)*scale),int((wx+26.0)*scale))
            
            
            # print(360-int(wy*30))
            # print(int(wx*30))
            # display.setColor(0xFFFFFF)
            # display.drawPixel(360-int(wy*30),int(wx*30))
            # display.setColor(0xFFFFFF)
            # display.drawPixel(2*m,m)
            # m += 1

    # Draw the robot's current pose on the 360x360 display
    # display.setColor(int(0xFF0000))
    # display.drawPixel(360-int(pose_y*scale),int(pose_x*scale))


    ###################
    #
    # Controller
    #
    ###################
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
            # Part 1.4: Filter map and save to filesystem
            np.save("map.npy",np.multiply(map_disaster>0.5,1))
            print("Map file saved")
            # print(np.multiply(map>0.5,1)[73,161])
        elif key == ord('L'):
            # You will not use this portion in Part 1 but here's an example for loading saved a numpy array
            map = np.load("map.npy")
            print("Map loaded")
        else: # slow down
            vL *= 0.75
            vR *= 0.75
    else: # not manual mode
        # Part 3.2: Feedback controller
        #STEP 1: Calculate the error
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
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)