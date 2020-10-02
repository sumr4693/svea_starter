# Generate arduino libraries for custom messages
# and add them to the arduino library folder.
# Assumes that the arduino libraries are stored in 
# ~/Arduino/libraries
source ./../../devel/setup.bash
cd ~/Arduino/libraries
NAME='_backup_ros_lib_'$(date '+%Y%m%d-%H%M%S')
cp -drf ros_lib ./../$NAME
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
