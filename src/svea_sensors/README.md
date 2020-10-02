# SVEA sensor pakcage
Contains settings and software for all default sensors of the SVEA vehicles

## Sensors

### ZED cam
Stereo camera for visual odometry.

### IMU
BNO055, 9 degrees of freedom IMU (can also give temperature).
The IMU is connected to the main computer through an i2c buss. Run `grant_i2c_permisions.sh` found in the root of the package directory if there are errors due to access rights when starting the IMU node.

### Hokyo Lidar
Connected through ethernet. Requires a little bit of setup.

### Wheel encoders
Not implemented

## Published Topics
This list is incomplete, since especially the zed camera publishes a lot of topics.
See the individual packages for complete descriptions. 

### /odom/filtered
Fused output from the EKF. Usefull for getting the velocity of the car.


## Notes

# Creating maps with slam and loading them for localization
To use a map for localization
Use `roslaunch svea_sensors slam.launch` to create a map. The map is saved to the .ros folder 
(will be hidden, use Ctrl + h if you can not find it) in 
the home directory when the slam node is terminated. The map files are named 
`mrpt_icpslam_*date*_*time*.simplemap`.

Load a map with `roslaunch svea_sensors localize.launch map_file:=*filepath to map here*`.
Ex to load the SML map use
`roslaunch svea_sensors localize.launch map_file:=$(find sml_sensors)/maps/sml.simplemap`


 



