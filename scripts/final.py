#!/usr/bin/env python3

from movement import back_up, set_movement_arm, set_vel, drive_to_target, set_arm, set_gripper
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


# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"


class Final:

    def __init__(self, robot_code):


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


    def do_actions(self):
        set_vel(self, 0,0)

        while(not self.inFront):
            find_and_face_color(self, "blue")
        drive_to_target(self)
        grip_and_lift(self)
        set_vel(self, 0,0)

        return True

    def run(self):
            rospy.spin()
            
if __name__ == '__main__':
    robot_code = sys.argv[1]
    follower = Final(int(robot_code))
    follower.run()
