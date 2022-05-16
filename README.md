# Final-Robotics-Project
Final project for introduction to robotics

Code is in algo.py and final.py.  We reused a lot of our code from Q Learning, but made it simpler and more reusable.

## Commands
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
rosrun image_transport republish compressed in:=raspicam_node/image raw out:=camera/rgb/image_raw