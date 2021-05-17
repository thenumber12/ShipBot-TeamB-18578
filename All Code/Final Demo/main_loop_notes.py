#include libraries
import time

#os.system('python3 gainSet.py')

#define class
class station:
    def __init__(self, wall_location, arm_pose, station_type, desired_position):
        self.wall_location = wall_location #coordinate from navigation code that matches station
        self.arm_pose = arm_pose #arm pose
        self.station_type = station_type #{V, S, B}
        self.desired_position = [desired_position] #desired position from mission file
        

    def get_wall_location(self):
        return self.wall_location
        
A = station('1', 'A', 'V', '')  
B = station('5', 'B', 'V', '')
C = station('4', 'C', 'V', '')
D = station('3', 'D', 'B', '')
E = station('7', 'E', 'S', '')
F = station('8', 'F', '', '')
G = station('2', 'G', 'S', '')
H = station('6', 'H', 'B', '')

stations = [A, B, C, D, E, F, G, H]
prio_list = sorted(stations, key=station.get_wall_location)


def main():
    #wall_location()  #replace with wall location func
    parse_missionfile()
    print("Execute Mission")
    for x in range(len(prio_list)):
        if (prio_list[x].desired_position != ''):
            arm_pos(prio_list[x].arm_pose)  #replace with arm pose func.
            wall_location(prio_list[x].wall_location)  #replace with wall location func.
        elif (prio_list[x].desired_position == ''):
            next()


#def wall_location()  #enter wall location func here
        
def parse_missionfile():
    file= open("mission_file_1.txt")
    list = file.readlines()
    num_lines = len(list) - 1
    print(num_lines)
    start_time = time.time()
    file= open("all_stations.txt")
    
    while file:
        count = 0
        prio_count = 0
        for line in file:
            print(line)
            count = count + 1
            prio_count = prio_count + 1
            line=line.split(" ")
            a=line[0]
            b=line[1]
##            if(len(line) >= 3):
##                c=line[2]
##                e=line[4]
##                g=line[6]
##            elif(len(line) <= 3):
##                c=0
##                e=0
##                f=0
                
            Get_desiredpos(a,b) #get desired position from mission file
            
            if (count == num_lines):
                print("End of Parsing Mission File")
                break
        break

               
def Get_desiredpos(a,b):
    if(a[0]== 'A'):
        A.desired_position = [b]
    elif(a[0]== 'B'):
        B.desired_position = [b]
##    elif(a[0]== 'D'):
##        D.desired_position = [c, e, g]
    elif(a[0]== 'C'):
        C.desired_position = [b]   
    elif(a[0]== 'E'):
        E.desired_position = [b]
    elif(a[0]== 'F'):
        F.desired_position = [b]
##    elif(a[0]== 'H'):
##        H.desired_position = [c, e, g]
    elif(a[0]== 'G'):
        G.desired_position = [b]
    

def wall_location(input): #replace with navigation/station detect code
    if (input == '1'):
        print("Going to location 1")
    if (input == '2'):
        print("Going to location 2")
    if (input == '3'):
        print("Going to location 3")
    if (input == '4'):
        print("Going to location 4")
    if (input == '5'):
        print("Going to location 5")
    if (input == '6'):
        print("Going to location 6")
    if (input == '7'):
        print("Going to location 7")
    if (input == '7'):
        print("Going to location 8")


def arm_pos(input): #replace with arm pos func.
    if (input == 'A'):
        print("Making arm pose A")
    if (input == 'B'):
        print("Making arm pose B")
    if (input == 'C'):
        print("Making arm pose C")
    if (input == 'D'):
        print("Making arm pose D")
    if (input == 'E'):
        print("Making arm pose E")
    if (input == 'F'):
        print("Making arm pose F")
    if (input == 'G'):
        print("Making arm pose G")
    if (input == 'H'):
        print("Making arm pose H")


main()

##def manipulate(s):
##   if(s.)     
##
##def main():
##    wall_location(); #fill the station list
##      #station.initial_status = []
##      #S {0, 1}, B {0 - 7}
##
##    execute_mission();
##       #navigate: X.location()
##       #arm_pos(X.arm_pose)
##           #X.desired_position
##              #if(D or G):
##                  #do some special case
##              #else
##       #prio + 1
##
##
##    for(int i = 0; i < prio; i++) {
##        manipulate(prio_list[i])


    


