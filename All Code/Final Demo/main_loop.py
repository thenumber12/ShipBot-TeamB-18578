#include libraries
import time
import serial
#import hebiTrajTest as htt

#os.system('python3 gainSet.py')

#define class
class location:
    #xyz
    #orientation
    def __init__(self, wall_location, valve_type, desired_position):
        self.wall_location = wall_location #coordinate from navigation code that matches station
        self.valve_type = valve_type
        self.desired_position = desired_position
    
    def get_location(self):
        return self.wall_location

        
A = location('A', '', [])  
B = location('B', '', [])  
C = location('C', '', []) 
D = location('D', '', []) 
E = location('E', '', [])  
F = location('F', '', [])  
G = location('G', '', [])  
H = location('H', '', [])  

arduino = serial.Serial(port='/dev/ttyACM0', baudrate = 115200)

prio_list = { 'A':A , 'B':B, 'C':C, 'D':D, 'E':E, 'F':F, 'G':G, 'H':H }
#pose1 = [.1, .1, .1, .1, .1]
#arm_pose = { 1: pose1, 2: pose2, 3: pose3, 4: pose4, 5:pose5 }
present_station = A

def parse_missionfile():
    stations = {}
    mission_file=open("mission_file_2.txt")
    list_instructions = mission_file.readlines()
    instructions = list_instructions[0].split(', ')
    station = ''
    for i in instructions[:-1]:
        if(i[1:2] != 'A' and i[1:2] != 'B'): #special case for breaker boxes
            station = i[:1]
            valve_type = i[1:3] #V1 = gate, V2 = large, V3 = shuttle
            desired_position = int(i[4:])
            
            current_station = prio_list.get(station)
            current_station.valve_type = valve_type
            current_station.desired_position += [desired_position]
            
        else:
            station = i[:1]
            valve_type = i[1:2] #we case on BreakerBox A or B here
            #Special Case Breaker Parsing
            breakers = [i[4:5], i[9:10], i[14:15]]
            temp_position = ['', '', '']
            possible_position = [i[6:7], i[11:12], i[16:17]]
            k = 0
            for j in breakers:
                if(j != ''):
                    temp_position[int(j)-1] = [possible_position[k]]
                k = k + 1
            
            current_station = prio_list.get(station)
            current_station.valve_type = valve_type
            
            if(not current_station.desired_position):
                current_station.desired_position = temp_position
            else:
                current_station.desired_position[0] += temp_position[0]
                current_station.desired_position[1] += temp_position[1][0]
                current_station.desired_position[2] += temp_position[2]
            
    
    for k in prio_list:
        curr = prio_list.get(k)
        if(curr.valve_type != ''):
            stations[curr.wall_location] = curr
    
    return stations


def write(x):
  arduino.write(bytes(x, 'utf-8'))
  time.sleep(0.05);
  print(x)
  return


def navigate(current_station):
  
    write(current_station.wall_location);
    arduino.flush()
    while 1:
      tdata = arduino.read(1)           # Wait forever for anything
      time.sleep(.1)              # Sleep (or inWaiting() doesn't give the correct value)
      
      if(tdata):
        print(tdata)
        break
      
    print("Arrived at: " + current_station.wall_location)
    return current_station
    

def main():
    #wall_location()  #replace with wall location func
    stations = parse_missionfile()
    #print_prio_list(stations)
    current_station = A
    for i in stations:
        current_station = navigate(stations.get(i))
        arduino.flush()
        time.sleep(10)
        #arm(current_station)
    
    
    print("Mission Complete")


def print_prio_list(stations):
    for i in stations:
        c_station = prio_list.get(i)
        print([c_station.wall_location,
               c_station.valve_type,
               c_station.desired_position,
               c_station.distance])

def realsense(current_station):
  xyz = [.1, .1, .1]
  orientation = 'h' #'h' horizontal, 'v' vertical
  self.current_station.xyz = xyz
  self.currnet_station.orientation = orientation
  """
def init_arm(current_station):
    realsense(current_station) #h, v
    if(current_station.valve_type == 'V1'):
        if(current_station.orientation == 'h'): 
            self.current_station.reference_pose = arm_pose.get(1)    
        elif(current_station.orientation == 'v'):
            self.current_station.reference_pose = arm_pose.get(2)  
		elif(current_station.valve_type == 'V2'): 
       self.current_station.reference_pose = arm_pose.get(1)
    elif(current_station.valve_type == 'V3'):
      if(current_station.orientation == 'h'): 
      			self.current_station.reference_pose = arm_pose.get(3) 
      elif(current_station.orientation == 'v'):
            self.current_station.reference_pose = arm_pose.get(4) 
    elif(current_station.valve_type == ('A' or 'B')): #get xyz
       self.current_station.reference_pose = arm_pose.get(5)    

        
def ik_arm(current_station):
  init_arm(current_station)
  #call to arm_trajectory
  #arm localization
  #in position
  
def manipulate(current_station):
	ik_arm(current_station);
  
  armTrajFunc(current_station.exec_trajectory)
  
  if(current_station.valve_type == ('V1' or 'V2'):
     #spin by current_station.desired_position
  elif (current_station.valve_type == 'V3'):
    if(current_station.orientation == 'h'):
        #all horizontal instruction
     		if (current_station.desired_position[0] == 1): 
            #move towards wall
        else:
            #move out of wall
		elif(current_station.orientation == 'v'):
        if (current_station.desired_position[0] == 1):
     				#move up
     		else:
     				#move down
    
     
	elif(current_station.valve_type == ('A' or 'B')):
     instruction = current_station.desired_position
     for i in range(len
  

"""
def arm_pose(input): #delete this
    if(input == 'Pose1'):
        print("pose1")
    elif(input == 'Pose2'):
        print("pose2")
    elif(input == 'Pose3'):
        print("pose3")
    elif(input == 'Pose4'):
        print("pose4")
    elif(input == 'up'):
        print("Up")
    elif(input == 'down'):
        print("Down")
                

main()


