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
* Creating new Vehicle object

```
def vehicle_initial_location(states)
```
* Initialized the Vehicle position


```
def close_path_point(states)
```
* Searching the whole path points for the closes one



```
def find_goal_point(states)
```
* This function return the index of goal point which the look ahead point is pointing to

```
def (states)
```
* 

```
def (states)
```
*

## Setup
To run this project:
* Use the next command for each main_* respectively and only from version main_04 and above.
* For optional assignment of 2D array, add file: “array.csv” to the same project path (only from version main_04).
* use cmd / Bash with the next command:

```
$ cd /c/.../"path library"
$ python main_*py -x0 0.1 -y0 0.1 -psi 0.0 -v 1.0
```
