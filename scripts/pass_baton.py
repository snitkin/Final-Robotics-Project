#!/usr/bin/env python3

from colorama import Back
from movement import back_up, set_movement_arm, set_vel, drive_to_target, set_arm, set_gripper, drive_to_basket
from movement import set_movement_arm, grip_and_lift, lower, find_and_face_ar, find_and_face_color, find_and_face_line, line_follower

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
import os
import pandas as pd
import numpy as np

import moveit_commander

import math
import sys

from final_robotics_project.msg import done_message
from final_robotics_project.msg import gripper_message

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class Final:

    # Make sure you're running with an int afterwards (1 or 2 or 3)
    # rosrun final_robotics_project pass_baton.py <1 or 2>
    def __init__(self, robot_code):
        self.objective_complete = False
        self.objective_complete2 = False  

        self.robot_code = robot_code
        
        self.inFront = False
        #intialize this node
        rospy.init_node("final")

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        cv2.namedWindow("window", 1)

        self.image_init = False
        print("not yet initialized")

        self.gripper_publisher = rospy.Publisher('gripper_action', 
            gripper_message, queue_size=20)


        self.gripper_subscriber = rospy.Subscriber('gripper_action',
            gripper_message, self.gripper_callback)
        

        # Publishers and subscribers
        if robot_code == 1:
            # Done publisher to publish when first leg completee
            self.done_publisher = rospy.Publisher('done_action', 
            done_message)

        elif robot_code == 2:
            # Done subscriber to find out when first leg complete
            self.done_subscriber =  rospy.Subscriber('done_action',
            done_message, self.done_callback)
            self.secured_baton = False

            #Done Publisher to publish when second lefe complete
            self.done_publisher = rospy.Publisher('done_action', 
            done_message)

        elif robot_code == 3:
            #Done subscriber to find out when second leef is completee
            self.done_subscriber =  rospy.Subscriber('done_action',
            done_message, self.done_callback)

        
        else:
            print("Robot code error: Must be 1 or 2 or 3")
            return


        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
            Image, self.image_callback)


        # subscribe to the robot's Laser scan data stream
        self.image_sub = rospy.Subscriber('scan',
            LaserScan, self.laser_scan)
        
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('cmd_vel', 
            Twist, queue_size=10)
        
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go([0,0,0,0], wait=True)

        # twist for movement
        self.twist = Twist(
            linear = Vector3(),
            angular = Vector3()
        )

        print("ready")

        self.min_arm_distance = 0.22
        self.min_drop_distance = 0.4
        
        self.image_init = True

        while not self.image_init:
            print("waiting for init")
        else:
            print("intialized")
            self.do_actions()     
    
    #image callback function
    def image_callback(self, msg):
        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        self.image_init = True

    #laser scan callback function
    def laser_scan(self, data):
        scan = np.array(data.ranges)
        #if value is 0, set to 60 because that's outside of range
        scan[scan == 0] = 60
        self.scan = scan

    # Helper function to cancel movement and arm angles
    def reset(self):
        set_vel(self, 0,0)
        set_gripper(self, [0,0])
        set_arm(self, [0,0,0,0])
        
    #main function to control actions for each robot
    def do_actions(self):
        #stop moving and reset arm and gripper
        self.reset()

        if self.robot_code == 1:

            self.min_arm_distance = 0.22
            
            #Goes to baton and picks up
            drive_to_target(self)
            grip_and_lift(self)
            
            r = rospy.Rate(1)
            #move forward into maze
            for i in range(2):
                set_vel(self,.1,0)
                r.sleep()
            
            #turn slightly so angled toward goal
            set_vel(self,0, - (np.pi / 4))
            r.sleep()
            
            #wait for maze to be completed
            a_star = False
            
            while not a_star:
                # wait for a star to complete, run on seperate computer
                a_star = input("Has Robot 1 completed the maze?")
            
            #move forward out of maze
            for i in range(2):
                set_vel(self,.1,0)
                r.sleep()
            set_vel(self,0,0)             
            #after finishing maze, lower baton
            r.sleep()
            lower(self)

            #publish message to done topic so robot 2 knows that first leg complete
            done = done_message(message="done 1")
            self.done_publisher.publish(done)

            print("Run Robot 2 code!\n")

            self.objective_complete = True
            
            # Wait for the other robot to grab the baton
            msg = False
            while not msg:
                msg = input("Has Robot 2 grabbed the baton?")

            #release baton and back up
            gripper_joint_goal = [0.018, 0.018]
            set_gripper(self, gripper_joint_goal)
            rospy.sleep(1)
            back_up(self)

            print("Robot 2 is good to lift")

        elif self.robot_code == 2:

            msg = input("Has Robot 1 lowered the baton?")

            #call done callback (ideally this would happen automatically after message published
            #however not possible given inability to run both robots on same computer
            #done callback has robot 2 move forward and grab baton
            done = done_message(message="done 1") 
            self.done_callback(done)

            rospy.sleep(2)
            
            # wait for robot 1 to release baton and back away
            msg1 = False
            while not msg1:
                msg1 = input("Has robot 1 backed up?")
            
            #call gripper callback (ideally this would also happen automatically) 
            #gripper callback lifts up baton after robot 1 has released
            done1 = gripper_message(message=msg1)
            self.gripper_callback(done1)

            rospy.sleep(2)
     
            #after lifting up baton, and repostioning move onto second task: driving to basket
            print("driving to basket")
            drive_to_basket(self)
            rospy.sleep(1)
            
            print("dropping in basket")
            lower(self)
            gripper_joint_goal = [0.018, 0.018]
            set_gripper(self, gripper_joint_goal)
            rospy.sleep(1)

            
            set_vel(self,0,0)
            
            self.objective_complete = True
            #publishes that second leg done so third robot can start
            done = done_message(message="done 2")
            self.done_publisher.publish(done)
            
            print("Run Robot 3 code!\n")

            #self.objective_complete after this
            self.objective_complete = True
            

        
        elif self.robot_code == 3:

            #wait for robot 2 to complete second leg
            msg = False
            while not msg:
                msg = input("Has robot 2 completed task?")
            done = done_message(message="done 2")
            
            #complete third and final leg
            self.done_callback(done)
            
            self.objective_complete = True    
            
        return True
    
    # Done callback, for robot 2, called when first leg is completed and must pick up baton
    def done_callback(self, msg):
        if msg = "done 1" and self.robot_code == 2:
            print("done callback")
            print(msg.message)

            # find and face orange baton
            find_and_face_color(self, "orange")

            # go to orange
            drive_to_target(self)

            # Pick up the baton
            gripper_joint_goal = [-0.007, -0.007]
            set_gripper(self, gripper_joint_goal)

            # Send publish to indicate that gripping baton and robot 1 can release
            gm = gripper_message(message="done")
            self.gripper_publisher.publish(gm)

            return
        elif msg = "done 2" and self.robot_code == 3:
            # done callback2 for when robot 2 has completed the second leg, called by robot 3
            print("done callback2")
            # drive foward to finish line 
            r = rospy.Rate(2)
            print("1")
            #drive forward
            for r in range(10):
                set_vel(self, .1,0)
                rospy.sleep(1)
            print("2")
            #happy dance!!!
            for i in range(4):
                set_vel(self,0,.35)
                rospy.sleep(1)
                set_vel(self,0,-.4)
                rospy.sleep(1)
                set_vel(self,0,0)
   
        else:
            print("error: not valid done message")
            
    # Gripper callback to coordinate baton eexchange
    def gripper_callback(self, msg):
        
        # for robot 1, called after robot 2 has grabbed baton so robot 1 must releease and backup
        if self.robot_code == 1:
            print("gripper callback 1")
            print(msg.message)

            gripper_joint_goal = [0.01, 0.01]
            set_gripper(self, gripper_joint_goal)
            self.time_to_drop = False

            back_up(self)
            self.objective_complete = True

            gm = gripper_message(message="done")
            self.gripper_publisher.publish(gm)

        # for robot 2, called after robot 1 has released baton so must lift baton up
        elif self.robot_code == 2:
            print("gripper callback 2")
            grip_and_lift(self)
            self.secured_baton = True

            #after lifting baton, robot 2 turns around from first robot
            r = rospy.Rate(1)
            for r in range(2):
                set_vel(self,0,(np.pi / 2))
                r.sleep()
            print("done turning")
            
            #move forward away from maze
            for i in range(2):
                set_vel(self,.1,0)
                r.sleep()
        return
    

    def run(self):
        rospy.spin()
            
if __name__ == '__main__':

    # Robot code is passed in as an argument to differentiate between bots
    robot_code = sys.argv[1]
    follower = Final(int(robot_code))
    follower.run()
