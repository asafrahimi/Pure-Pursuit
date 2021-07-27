import matplotlib
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.pyplot as pl
from argparse import ArgumentParser
import csv
import os 

#--------for getting inputs x0, y0, psi, v-------
parser = ArgumentParser()
parser.add_argument("-x0", help="The first arg named x0")
parser.add_argument("-y0", help="The seconed arg named y0")
parser.add_argument("-psi", help="The seconed arg named psi")
parser.add_argument("-v", help="The seconed arg named v")
args = parser.parse_args()

x0 = float(args.x0)
y0 = float(args.y0)
psi = float(args.psi)
v = float(args.v)
#-------------------------------------------------

dt = 0.05 #[s]
time = 0.0 #current simulation time
simulation_time = 200 #[s]
S_t = 0.2 # Servo delay 200 [msec]
L = 2.728 # length of Ford Fusion car [meter]
#v = 6 # car velocity [m/s]
K = 5 #proportion controller for look ahead [sec]
Lr = K*v + L #[m] Ld required gain
Ld = 0.0 #actual look ahead length
ind = 0.0 #global index
max_delta = 0.5 #max wheel angle [rad]
min_delta = -0.5
delta_ref = 0.0 #required delta [rad]
last_delta_ref = 0.0
delay = 0.2 #[s]
a = 0.0 #for servo dynamic function
c_x = 0.0 # look ahead x point
c_y = 0.0 # look ahead y point
global i # lidar dataset index
servo_bias = 0.03#[rad] 


#----check if there is a csv file in the path directory:-----
if (os.path.exists('array.csv')):
    # Read CSV file
    cx = []
    cy = []
    with open('array.csv', 'r') as csv_file:
        csv_reader = csv.reader(csv_file)

        for row in csv_reader:
            cx.append(float(row[0]))  
            cy.append(float(row[1]))
            
else: #define waypoints in case no such a csv file
    cx = np.linspace(0,1200,200) #define x waypoints
    #cy = 0.1*cx*np.sin(cx) #define y waypoints
    cy = np.linspace(0,1200,200) # i define here linear line to highlight the bias phenomena 
#-------------------------------------------------------------

class VehicleStates: #for create vehicle object 

    def __init__(self, Xg, Yg, v, psi, delta, omega): #vehicle attributes
        self.Xg = Xg
        self.Yg = Yg
        self.v = v
        self.delta = delta
        self.omega = omega
        self.pai = psi

#-----------------Pursuit Algorithm----------------------

def vehicle_initial_location(states): #1 Vehicle initial conditions
        states.Xg = x0 #[m]
        states.Yg = y0 #[m]
        states.v = v #[m/s]
        states.delta = 0.0 #[rad]
        states.omega = 0.0 #[rad/s]
        states.psi = psi #[rad]
        return states     

def closet_path_point(states): #2 finding the closes path point
    global ind
    i=0 #index
    Cp=0 #closees path point [m]
    while (i < len(cx)): #searching the whole path points
        dx = cx[i] - states.Xg
        dy = cy[i] - states.Yg
        d = math.sqrt(dx**2 + dy**2)
        if (i==0) or (Cp>d):
            Cp = d
            ind = i
        i+=1
    return states

def find_goal_point(states): #3 find the goal point
    global ind
    global c_x, c_y
    j = ind #index
    b = False #boolean True when we find look ahead point
    while (j < len(cx)) and (b==False):
        dx1 = cx[j] - states.Xg
        dy1 = cy[j] - states.Yg
        d1 = math.sqrt(dx1**2 + dy1**2)
        dx2 = cx[j+1] - states.Xg
        dy2 = cy[j+1] - states.Yg
        d2 = math.sqrt(dx2**2 + dy2**2)
        if (d1 < Lr) and (d2 < Lr): #in case the current and next points are too close
            j+=1
            c_x = cx[j+1]
            c_y = cy[j+1]
        elif ((d1 < Lr) and (d2 > Lr)): #in case the current point close than the required look ahead length and the next point is too far
            b = True
            c_x = (cx[j+1] + cx[j])/2 #interpolation
            c_y = (cy[j+1] + cy[j])/2 #interpolation
            ind = j+1
        elif (d1 > Lr) and (d2 > Lr): #in case there is no closer new look ahead point 
            b=True
            ind = j
            c_x = cx[j]
            c_y = cy[j]
        else:
            b=True
    return c_x, c_y

def white_noise(std):#4 adding white noise function to heading angle and vehicle position
    mean = 0
    std = std  
    num_sample = 1
    sample = np.random.normal(mean, std, size=num_sample) #random values using normal distribution
    return sample

def set_steering(states): #5 calculate the new required steering angle
    global Ld
    global ind
    global delta_ref
    dx = c_x - states.Xg
    dy = c_y - states.Yg  

    Ld = math.sqrt(dx**2 + dy**2)

    states.alpha = math.atan(dy / dx) - states.psi

    delta_ref = math.atan(2*L*math.sin(states.alpha) / Ld)

    return delta_ref, ind

def steady_state_bias():# this function insert steady state bias to servo
    global delta_ref
    delta_ref = delta_ref + servo_bias    
    return 

def wheel_delay(states): #6 for simulating servo dynamics
    global delta_ref
    global last_delta_ref
    global a
    if (last_delta_ref != delta_ref):
        a = delta_ref - states.delta 
        last_delta_ref = delta_ref

    if (states.delta != delta_ref) and (states.delta<max_delta) and (states.delta>min_delta): #update wheel angle if true 
        states.delta = states.delta + a*dt/delay
    return states

def update_vehicle_position(states, delta): #7 update vehicle states every loop with this function
    states.delta = delta
    states.psi = states.psi + dt*states.v*math.tan(states.delta)/L + white_noise(0.0174) #std of 1 deg (0.0174 rad) white noise
    states.Xg = states.Xg + dt*states.v*math.cos(states.psi) + white_noise(0.1) #std of 0.1 meter white noise
    states.Yg = states.Yg + dt*states.v*math.sin(states.psi) + white_noise(0.1) #std of 0.1 meter white noise
    return states

class LidarDataSet: #for create lidar object 

    def __init__(self, R_lidar, Phi_lidar): #lidar attributes
        self.R_lidar = R_lidar #[m]
        self.Phi_lidar = Phi_lidar #[rad]
    
def lidar(dataset1, dataset2):#8 save only stationary points (check if stationary by doppler)
    dr = abs(dataset2.R_lidar - dataset1.R_lidar)
    return dr

def kalman_filter():#9 do sensor fusion from the lidar and rest of the sernsors

    return

def main():#10 run the whole simulation
    global ind
    global time
    states = VehicleStates(Xg=x0, Yg=y0, v=v, delta=0.0, omega=0.0, psi=psi)
    states = vehicle_initial_location(states)
    if ind==0:
        states = closet_path_point(states)

    while  (time < simulation_time) and (ind < len(cx)): #make the iterations with time delta of dt between every loop
        find_goal_point(states)
        calc_delta = set_steering(states)
        steady_state_bias()
        wheel_delay(states)
        states = update_vehicle_position(states, states.delta)
        time = time + dt
        real_path, = plt.plot(states.Xg, states.Yg, color = 'blue', marker = ".")
        #plt.legend([real_path], ['real path'])
        look_ahead_poinnts, = plt.plot(cx[ind],cy[ind], color = 'red', marker = "o")
        if (time == dt):  #insert in loop just for the legend merging
            desired_path, = plt.plot(cx, cy, color = 'green')
        plt.legend([real_path, look_ahead_poinnts, desired_path], ['Real path', 'Look ahead points', 'Desired path'])
        #print(states.delta, delta_ref)


main()
plt.title('Pure Pursuit')
plt.xlabel('Xg [m]')
plt.ylabel('Yg [m]')
plt.grid()
plt.show()

    
