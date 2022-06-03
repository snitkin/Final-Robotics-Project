# Final-Robotics-Project
Final project for introduction to robotics

Code is in algo.py and final.py.  We reused a lot of our code from Q Learning, but made it simpler and more reusable.

# Project Description
The goal of this project is to have the robot complete a relay race. This project incorporates many of the different algorithms and topics that we learned throughout the quarter. Due to this, we thought it would be a good way to tie what we have learned into a single project in a fun and visual way. The first main component of this project is to incoporate A* path finding algorithm to have the robot sucessfully navigate through a maze with a baton in hand. Once the robot makes it out of the maze it will have to pass the baton to another robot that meets it at the exit of the maze. Using user input the robots communicate with eachother to pass the baton. The second robot with the baton in hand will continue the relay, following along a line until it reaches the end and activates a third robot that will make it to the finish line. 

Link to video demo:
[video](https://drive.google.com/file/d/1eA4pDETtZJvgu-7c6HNZG-yaTawgXvTW/view?usp=sharing)

Gif:

![gif](https://github.com/snitkin/Final-Robotics-Project/blob/b13ae6a7a652c7ea174794ceef793f2b673d556c/FinalProject.gif)

# System Architecture
## Path Finding

Path finding uses the A star algorithm in algo.py. Three arrays are generated that are the same dimensions as the map: the open array, the closed array, and node info. Iterating through the a star algorithm from a hardcoded start node, the node info is calculated and nodes are added and removed from the other two arrays as appropriate. When the hardcoded end node is reached, a path is calculated by tracking parent nodes through the closed list. Using this path, waypoints are claculated by iterating through and finding when changes in x and y coordinates are sufficiently big enough to constitute a new way point.


## Navigation

The code for navigation can be found in algo.py in move_along_path() and moveToGoal(). Once the waypoints have been generated, we iterate along the path for each goal_pose. Each goal_pose is added to the action client, which reads the goal to move the robot there. The action client waits until the goal has been reached, at which point it returns that the status of the goal is completed and the function moves on to the next goal pose.

## Baton Pass and Race to Finish Line

The code for the baton pass can be found in pass_baton.py all within the do_action() function where many of the functions inside are then imported from movement.py. If pass_baton.py is run with argument value 1 then the code will first pick up the baton then publish a done message when the robot has completed the maze. The second robot can be called upon when running pass_baton.py with argument value 2. The user will then be prompted with a question to see if robot one has lowered the baton. Robot 2 will then pick up the baton then turn to drop it off in in a basket. At this point robot three can be run by running pass_baton.py with argument value 3



# Execution
## Path Finding (Sam)
- once turtlbot is roscore and brought up, 
place it at the start of the maze (closer to CSIL 5) facing the goal
- roslaunch final_robotics_project turtlebot3_navigation.launch
- rosrun final_robotics_project algo.py

## Baton Pass Commands (Matthias)
- roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
- roslaunch turtlebot3_manipulation_moveit_config move_group.launch
- rosrun image_transport republish compressed in:=raspicam_node/image raw       out:=camera/rgb/image_raw
- rosrun final_robotics_proect final.py ([1] or [2] or [3])

# Challenges

## Relay
One of the main challanges with the second part of the relay was trying to incorporate new components that we havent worked with before. One of the desired obstacles was trying to get the robot to go up a ramp. We found that when we tried to teleop the robot up the ramp it wouldnt be feasible unless the robot was coming in at an angle while maintaining an accelerating velocity. Once it did make it up the ramp due to the friction difference the robot would often make it our of the sides. As a result we decided to resort to simpler relay activities like droping the baton in the basket. 
## Pass the Baton 
A challeng with passing the baton included trying to get the arm to function by sending messages to th different robots. We werent able to solve this but resorted to using useer input to communicate between the robots instead. 

## Orinetation

Our map was flipped in RVIZ, and thinking about how to calculate anlges from multiple coordinate planes was incredibly confusing and took more time than anticipated. 

## Navigation in the Maze

The AMCL package did not like navigation in the maze we had previously built. The marigins of error were too small, it was afraid it would run in to things and stop navigating. To solve this, we simplified the maze. Unfortunately, the simplified maze is less interesting.

# Future Work
## Pass the Baton
If we had more time it would haeve been intresting to explore other relay options including finding a different material for the ramp to minimizee the friction consideration.
It also would have been intresting to try and get the messaging between the robots to work. We were given an option to explore regarding the launch file so upon more time we could have tried to figure out how to make it work. 

## Built in Navigation

Given more time, we would love to not have to rely on the naviagtion system of AMCL but rather to build our own tools for navigation using robot odometry. Hopefully this would allow us to navigate through more narrow passageways.


# Takeaways

One big takeaway from this project was learning to comprimise. There were lots of things we wanted to include in this project that were too difficult, time consuming, or literally impossible to implement. We had to choose which parts to cut and simplify and what elements of the robot we were not willing to comprimise on. This is a huge skill for future large sclae project work. 


## Senior Contributions
### Matthias
My contributions were mainly twofold; making the mechanics of the baton pass work and making our code setup reusable and modular.  I first took a lot of the structure from QLearning but made it more straightforward.  I noticed that the pass_baton field was too long (like in our q_learning) so I moved all the movement functions to movement.py.  That made the code easier to read, debug, and change down the road.

I also designed a setup for message communication between the robots.  I set up a "gripper" and "done" message for robots to tell their partners that they were good to pick up the baton and go.  This was ultimately unsuccessful due to unforseen errors, but the setup of the publishers and subscribers is all there and I was able to refactor the code to work on two machines based on user input.  The demo is here:
https://drive.google.com/file/d/1iCZ_ChCNZ_JuhM51o3zSL4-INP2QT63O/view?usp=sharing 

The original design was for the first robot to finish the maze and then broadcast a "done" message for the other robot to come find it.  Then the second robot would grip the baton and publish a gripper message to let the first one know to let go and back up.  Once that happened the first robot would reply with a gripper message letting the second know that it could lift.  Then the second robot would lift the baton and proceed with the next tasks.

### Sam

For this project I largely contributed to the path finding and localization steps. I am turning in a robot that can start in a known maze and use the A star alogirthm to find the best path to the end, and then naviagte along that path to get out of the maze. All of the code I worked on can be found in scripts/algo.py as well as the launch files visualize_maze.launch and turtlebot3_navigation.launch

Demo: https://youtu.be/48Zm1apcv1Q

