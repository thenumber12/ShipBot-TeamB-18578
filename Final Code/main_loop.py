
import time
import serial
import sys




#define class
class location:

    def __init__(self, wall_location, valve_type, desired_position):
        self.wall_location = wall_location
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

def parse_missionfile(file):
    stations = {}

    mission_file=open(file)
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
      tdata = arduino.read(1)  # Wait forever for anything
      time.sleep(.1)           # Sleep (or inWaiting() doesn't give the correct value)
      
      if(tdata):
        print(tdata)
        break
      
    print("Arrived at: " + current_station.wall_location)
    return current_station
 

def print_prio_list(stations):
    for i in stations:
        c_station = prio_list.get(i)
        print([c_station.wall_location,
               c_station.valve_type,
               c_station.desired_position])


def main():
    if(len(sys.argv) > 1):
        file = sys.argv[1]
    else:
        file = 'all_stations.txt'
    
    stations = parse_missionfile(file)
    print_prio_list(stations) #uncomment for list of mission file dictionary
    current_station = A
    for i in stations:
        current_station = navigate(stations.get(i))
        arduino.flush()
        #time.sleep(10)
        
    
    
    print("Mission Complete")


main()


