#!/usr/bin/env python3

from movement import back_up, set_movement_arm, set_vel, drive_to_target, set_arm, set_gripper, find_and_face_robot
from movement import set_movement_arm, grip_and_lift, lower, find_and_face_ar, find_and_face_color

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

    # Make sure you're running with an int afterwards (1 or 2)
    # rosrun final_robotics_project final.py <1 or 2>
    def __init__(self, robot_code):

        self.objective_complete = False

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
            gripper_message)


        self.gripper_subscriber = rospy.Subscriber('gripper_action',
            gripper_message, self.gripper_callback)

        # Publishers and subscribers
        if robot_code == 1:
            # Done publisher
            self.done_publisher = rospy.Publisher('done_action', 
            done_message)

        elif robot_code == 2:
            # Done subscriber
            self.done_subscriber =  rospy.Subscriber('done_action',
            done_message, self.done_callback)
            self.secured_baton = False

        
        else:
            print("Robot code error: Must be 1 or 2")
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

        while not self.image_init:
            print("waiting for init")
        else:
            print("intialized")
            self.do_actions()
            #if one then for the first robot finding second robot
            #if two then for second robot finding ar tag??

            # self.robot_control()

    # Done callback, for robot 2
    def done_callback(self, msg):
        # Go and find the color
        print("done callback")
        print(msg.message)
        
        # find and face orange
        find_and_face_color(self, "orange")

        # go to orange
        drive_to_target(self)


        # Pick up the thing
        gripper_joint_goal = [-0.007, -0.007]
        set_gripper(self, gripper_joint_goal)

        # Send publish
        gm = gripper_message(message="done")
        self.gripper_publisher.publish(gm)

        self.secured_baton = True
        return

    # Gripper callback, for robot 1
    def gripper_callback(self, msg):
        
        # If code == 1 then let go, back up 
        if self.robot_code == 1:
            print("gripper callback")
            print(msg.message)

            gripper_joint_goal = [0.01, 0.01]
            set_gripper(self, gripper_joint_goal)
            self.time_to_drop = False

            back_up(self)
            self.objective_complete = True

            gm = gripper_message(message="done")
            self.gripper_publisher.publish(gm)        
        # If code == 2 - 1 has let go, lift
        else:
            grip_and_lift(self)

        # return

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

    #i played with this
    def do_actions(self):
        set_vel(self, 0,0)


        if self.robot_code == 1:

            a_star = True
            
            while not a_star:
                # Do the damn a star
                i = 1
            
            lower(self)

            done = done_message(message="done")
            self.done_publisher.publish(done)

            # Wait for the other robot to grab the baton
            while not self.objective_complete:
                i = 1

        elif self.robot_code == 2:
            
            # by the time we exit this loop, 2 has the baton over its head
            while not self.secured_baton:
                i = 1

            self.objective_complete = True

            while not self.objective_complete:
                i = 1



        # color_options = ["pink","orange"]
        # color_choice = np.random.choice(color_options)
        # while(not self.inFront):
        #     if color_choice == "pink":
        #         find_and_face_color(self,"pink")
        #         drive_to_target(self)
        #         grip_and_lift(self)
        #         #need hard coded arm values for pink 
        #         #my rviz doesnt load so i couldnt get value
        #     if color_choice == "orange":
        #         find_and_face_color(self,"orange")
        #         drive_to_target(self)
        #         grip_and_lift(self)
        #         #hard coded values for orange 
 
        # set_vel(self, 0,0)
    
        return True

    #Things we need to decide if 1) we are just gonna have the first robot find the second robot which is at a set position
    #2)once the first robot starts moving then we have it send a message to the second robot to get into place
    #3)Im thinking the first robot finds the second robot through color finding we put orange tape on its perimeter
    #4)How to communicate to the second robot that the first robot is infront and it can execute to pick up baton
    #5)once it picks up the button are we putting ar tag on? we can brainstorm this one
    
    #function for first robot to find second robot and for second robot to grab and go somewhere
    def robot_control(self,number):
        #call move_robo2_to_position sometime before
        if number == 0:
            find_and_face_robot(self,"orange")
            drive_to_target(self)
            lower(self)
        if number == 1:
            #lower_robot2_arm(self)
            grip_and_lift(self)
            find_and_face_ar(self)
            lower(self)


            
        



    def run(self):
        rospy.spin()
            
if __name__ == '__main__':
    robot_code = sys.argv[1]
    follower = Final(int(robot_code))
    follower.run()
