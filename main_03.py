import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import animation 

dt = 0.05 #[s]
time = 0.0
OG = [0,0,0] # origin on axes
S_t = 0.2 # Servo delay 200 [msec]
L = 2.728 # length of Ford Fusion car [meter]
v = 6 # car velocity [m/s]
Lr = 15+L #[m] Ld required gain
Ld = 0
ind = 0 #global index
max_delta = 0.5 #max wheel angle [rad]
min_delta = -0.5
delta_ref = 0 #[rad]
last_delta_ref = 0
delay = 0.2 #[s]
a = 0 #for servo dynamic function
c_x = 0 # look ahead x point
c_y = 0 # look ahead y point


cx = np.linspace(0,1200,200) #define x waypoints
cy = 0.1*cx*np.sin(cx) #define y waypoints


class VehicleStates:

    def __init__(self, Xg, Yg, v, psai, delta, phi):
        self.Xg = Xg
        self.Yg = Yg
        self.v = v
        self.delta = delta
        self.phi = phi
        self.psai = psai

#------------Pursuit Algorithm-----------

def vehicle_initial_location(states):#1
        states.Xg = 0.0 #[m]
        states.Yg = 0.0 #[m]
        states.v = 6 #[m/s]
        states.delta = 0.0 #[rad]
        states.phi = 0.0 #[rad]
        states.psai = 0.0 #[rad]
        return states     

def closet_path_point(states): #2 finding the closet path point
    global ind
    i=0 #index
    Cp=0 #closet path point [m]
    while (i < len(cx)):
        dx = cx[i] - states.Xg
        dy = cy[i] - states.Yg
        d = math.sqrt(dx**2 + dy**2)
        if (i==0) or (Cp>d):
            Cp = d
            ind = i
        i+=1
    return states

def find_goal_point(states): #3 find thr goal point
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

def set_steering(states): #4
    global Ld
    global ind
    global delta_ref
    dx = c_x - states.Xg
    dy = c_y - states.Yg  

    Ld = math.sqrt(dx**2 + dy**2)

    states.alpha = math.atan(dy / dx) - states.psai

    delta_ref = math.atan(2*L*math.sin(states.alpha) / Ld)
    return delta_ref, ind

def wheel_delay(states): #5servo dynamics
    global delta_ref
    global last_delta_ref
    global a
    if (last_delta_ref != delta_ref):
        a = delta_ref - states.delta 
        last_delta_ref = delta_ref

    if (states.delta != delta_ref) and (states.delta<max_delta) and (states.delta>min_delta):
        states.delta = states.delta + a*dt/delay
    return states

def update_vehicle_position(states, delta): #6
    states.delta = delta
    states.psai = states.psai + dt*states.v*math.tan(states.delta)/L
    states.Xg = states.Xg + dt*states.v*math.cos(states.psai)
    states.Yg = states.Yg + dt*states.v*math.sin(states.psai)
    return states

def main():#7
    global ind
    global time
    states = VehicleStates(Xg=0.0, Yg=0.0, v=0.0, delta=0.0, phi=0.0, psai=0.0)
    states = vehicle_initial_location(states)
    if ind==0:
        states = closet_path_point(states)

    while  (time < 200) and (ind < len(cx)): 
        find_goal_point(states)
        calc_delta = set_steering(states)
        wheel_delay(states)
        states = update_vehicle_position(states, states.delta)
        time = time + dt
        plt.plot(states.Xg, states.Yg, color = 'blue', marker = "*")
        plt.plot(cx[ind],cy[ind], color = 'red', marker = "o")
        print(states.delta, delta_ref)


main()
plt.plot(cx, cy, color = 'green', marker = ".")
plt.title('Pure Pursuit')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.grid()
plt.show()

    
