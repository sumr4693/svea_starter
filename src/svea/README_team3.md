# Overview
This folder contains all the nodes and algorithms implemented by the group Spice Level V on the SVEA 3 car. The group consists of the following group members:

* Mustafa Al-Janabi
* Mikel Zhobro
* Karl Hemlin
* Subramanian Murali Ram
* Mikael Glamheden

# Group contribution
In the following table all the different parts the group has worked on throughout the project are highlighted. The group member that contributed to the specific task is marked with a checkmark.

|                               | Mustafa | Mikel | Karl | Subramanian | Mikael |  
|:---|:---:|:---:|:---:|:---:|:---:|
| Path generation               |  |   |   |   |  ✓  |
| Emergency brake               | ✓ | ✓  |  ✓ |   |   |
| Speed control                 | ✓  | ✓  | ✓  | ✓  | ✓  |
| Tune pure pursuit for racing    |   |   |  ✓ |  ✓ | ✓  |
| Path replanner                |  ✓  |  ✓  |   |  |  |
| Obstacle avoidance            |  ✓ | ✓  |   | ✓  |   |
| Geofence & donuts             |   |   | ✓  |  ✓ | ✓  |

# Implementation details
The following methods and features have been implemented in the cars

### Speed control
* XY Linear velocity as feedback
* PID controller with anti-wind up

### Emergency brake
* Dynamic emergency distance based on speed
* Dynamic emergency angle based on steering angle

### Track Race
* Waypoints creation
* Dubin’s path for path generation
* Pure pursuit for path tracking
* Works with Emergency brake and Speed limit
* Initial position is selected in Rviz

### Obstacle avoidance
* Obstacle map with 2 seconds memory
* Dynamic A* for partial planning
* Future path checking
* For closer obstacles - Ability to back up car multiple times and plan path for longer time period

### Donut race
* Waypoints from points in circle
* Cubic spline for path generation
* LQR for path tracking
* Centre point is selected in Rviz
* Geofence stops car if outside

# Launching
In this sections the steps to launch each part of the challenge are described in simple steps. All parts require an ssh connection with the TX2 on the car. This is necessary to be able to start the launch files remotely. Furthermore it is necessary to make sure that the ROS Master of the car is broadcasting to all devices on the WIFI-network of the car. On the car this is achieved by running
```bash
. /svea_starter/scripts/export_ros_ip.sh
```
On the the local computer make sure to execute any of
```bash
. /svea_starter/scripts/export_ros_ip_NAME.sh
```
and make sure to change the HOSTNAME to the one corresponding to your own machine.

## Racing
To initate the racing mode do the following

**_On the TX2 (via SSH)_**

1. ```bash
    roslaunch svea floor2_pure_pursuit.launch
    ```

**_On local machine_**

0. Make sure connection is setup correctly, try to list all ROS topics and see if they match the ones on the car.
1. Start _rviz_
2. Click on _2D Pose Estimate_ and give the car its initial position
3. Place the car on the start line
4. Make sure the remote is NOT in override
5. Click on _Publish Point_ and publish a point anywhere to give start signal.

The car will start the race around the track on floor 2.

## Obstacle avoidance
Firstly make sure the car remote is in **override mode**

**_On the TX2 (via SSH)_**

1. ```bash
    roslaunch svea floor2_pure_pursuit_obstacle_avoidance.launch
    ```

**_On local machine_**

1. Start _rviz_
2. Click on _2D Pose Estimate_ and give the car its initial position
3. Place the car on the start line
4. Take the car remote off override

The car will start the driving around the obstacle course.

## Donuts

**_On the TX2 (via SSH)_**

1. ```bash
    roslaunch svea floor2_donut_lqr_real.launch
    ```

**_On local machine_**

0. Make sure connection is setup correctly, try to list all ROS topics and see if they match the ones on the car.
1. Start _rviz_
2. Click on _2D Pose Estimate_ and give the car its initial position
3. Click on _Publish Point_ and publish a point that is going to be the circle's and geofence's centre.
4. Place the car on the start line
5. Make sure the remote is NOT in override
6. Click on _Publish Point_ and publish a point anywhere to give start signal.


# See also
Epic video of our car

<a href="http://www.youtube.com/watch?feature=player_embedded&v=mqogOQvzAho
" target="_blank"><img src="http://img.youtube.com/vi/mqogOQvzAho/0.jpg"
alt="Epic video link" width="240" height="180" border="10" /></a>
