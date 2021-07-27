# Pure-Pursuit Algorithm
* [General info](#general-info)
* [Technologies](#technologies)
* [Program Functions](#Program-Functions)
* [Setup](#setup)

## General info
Pure pursuit is a tracking algorithm that works by calculating the curvature that will move a vehicle from its current position to some goal position.
The whole point of the algorithm is to choose a goal position that is some distance ahead of the vehicle on the path..
	
## Technologies
Project was created with languages, libraries and versions:
* Python version 3.9.6
* matplotlib
* numpy as np
* math
* matplotlib.pyplot as plt
* matplotlib.pyplot as pl
* ArgumentParser
* csv
* os 
	
## Program Functions
Program Classes and Functions and their Functionality:

```
class VehicleStates
```
* Creating new Vehicle object.

```
def vehicle_initial_location(states)
```
* Initialized the Vehicle position.


```
def close_path_point(states)
```
* Searching the whole path points for the closes one.

```
def find_goal_point(states)
```
* This function returns the index of goal point which the look ahead point is pointing to.

```
def wgite_noise(std)
```
* Insert random normal distribution of white noise to heading angle.

```
def set_steering(states)
```
* This function returns the calculating current required steering angle (delta_ref).

```
def steady_state_bias()
```
* This function insert constant steady state bias to actuator/servo wheel.

```
def wheel_delay(states)
```
* This function is supposed to simulate the wheel servo dynamics delay response .

```
def update_vehicle_position(states, delta)
```
* This function is going to update the vehicle position states with every simulation iteration.

```
class LidarDataSet
```
* This class creates new Lidar object, which supposed to simulate inputs from Lidar, and convert them to sets of dataset, which after, help us to make the more robust tracking.

```
def lidar(dataset1, dataset2)
```
* This function is supposed to simulate doppler effect filter, which returns only the stationary points.

```
def kalman_filter(states)
```
* This function supposed to simulate the Kalman Filter Algorithm, 
that uses a series of measurements observed over time, 
containing statistical noise and other inaccuracies,
and produces estimates of unknown variables (our states)
that tend to be more accurate than those based on a single measurement alone.

```
def main(states)
```
* This function run the whole simulation functions.


## Setup
To run this project:
* Use the next command for each main_* respectively and only from version main_04 and above.
* For optional assignment of 2D array, add file: “array.csv” to the same project path (only from version main_04).
* use cmd / Bash with the next command:

```
$ cd /c/.../"path library"
$ python main_*py -x0 0.1 -y0 0.1 -psi 0.0 -v 1.0
```
