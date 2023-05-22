# A-Star-for differential-drive-robot

Dependencies required:
* Ubuntu 20.04
* Gazebo
* ROS Noetic
* turtlebot3 packages
* Python packages: Numpy, opencv, queue, time

libraries used in this algorithm
* numpy
* opencv
* heapq
* time
* rospy
* geometry_msgs

# Stepts to run the algorithm
1. Paste the a_star_diff_drive in the src folder of ros workspace
2. build the package using catkin_make
3. In one terminal, paste : "roslaunch a_star_diff_drive a_star.launch", once the gazebo is launched, in another terminal, enter into the directory catkin_ws/src/a_star_diff_drive/, and execute "python3 source_code.py"
4. Enter the clearance value, start position, goal position and RPM values.
5. Once the goal is reached, the node exploration and path will be visualised using opencv.
6. After the visualization, the turtlebot3 will start moving to the goal location.

# Test cases used:
1. start: 50 100 0, goal: 500 100, clearance: 5, RPM: 10,20
2. start: 50 100 0, goal: 400 50, clearance: 5, RPM: 10,20

project members: 
1. Sai Surya
Directory ID - saisurya
UID -119224113

2. Dhruv Sharma
Directory ID - dhruvsh
UID - 119091586

Github Repo - https://github.com/Surya-Sriramoju/A-star-for-differential-drive-robot

link to the video: 
* Part 1: https://drive.google.com/file/d/1YUwDJA55Mi58GMRdZ81Du392SIRcXdfm/view?usp=sharing
* Part 2: https://drive.google.com/file/d/1XgQOXYi0GzfUdhCxMrqEMVNwKaZDNoDb/view?usp=sharing

