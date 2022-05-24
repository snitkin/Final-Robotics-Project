
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
import os
import pandas as pd
import numpy as np

import moveit_commander

import math
import sys

kp = 0.05


# sets the robot linear and angular velocity
# uses self.twist
def set_vel(self, linear, angular):

    self.twist.linear.x = linear
    self.twist.angular.z = angular
    self.robot_movement_pub.publish(self.twist)

# Uses proportional control to drive to a target based on self.scan 
# looks at 10 degrees in either direction
def drive_to_target(self):

    set_movement_arm(self)

    bound1, bound2 = -10, 11

    m = 100
    while m > self.min_arm_distance:

        scan = list(self.scan)
        arr = scan[bound1:] + scan[:bound2]
        m = min(arr)

        index = arr.index(m)

        #-9 instead of -10 to compensate for listing to the right
        index -= -(bound1 + 1)

        angular = (kp * index / 2)

        set_vel(self, 0.05, angular)
        rospy.sleep(0.1)  

    print("done with while")

    set_vel(self, 0,0)
    rospy.sleep(1)

# Sets arm to set of given params
def set_arm(self, goal):
    goal = map(math.radians, goal)
    self.move_group_arm.go(goal, wait=True)
    self.move_group_arm.stop()
    rospy.sleep(2)

# Sets gripper to a set of given params
def set_gripper(self, goal):
    self.move_group_gripper.go(goal, wait=True)
    self.move_group_gripper.stop()
    rospy.sleep(2)

# Sets the arm in a position to move forward and pick up a tube
# Can adjust for picking other stuff up
def set_movement_arm(self):
    gripper_joint_goal = [0.018, 0.018]
    set_gripper(self, gripper_joint_goal)

    goal = [0,5,0,0]
    set_arm(self, goal)
    print("gripper in position")
    rospy.sleep(2)

# Grips and lifts an object above the Lidar
def grip_and_lift(self):
    #want to determine the height of the pixel (y) at a set distance x 
    if self.robot_code == 1:
        goal = [0,30,0,0]
    else:
        goal = [0,-50,0,0]
    set_arm(self, goal)
    print("gripper in position")

    print("lifting")
    gripper_joint_goal = [-0.007, -0.007]
    set_gripper(self, gripper_joint_goal)

    if self.robot_code == 1:
        goal = [0,-50,0,0]
    else:
        goal = [0,-50,0,0]
    set_arm(self, goal)
    rospy.sleep(2)
    print("lifted")

# Lowers object
def lower(self):
    print("putting")
    goal = [0,0,0,0]
    set_arm(self, goal)

    # gripper_joint_goal = [0.01, 0.01]
    # set_gripper(self, gripper_joint_goal)
    # self.time_to_drop = False
    # print("put")

def back_up(self):
    twist = Twist(
        linear = Vector3(),
        angular = Vector3()
    )

    twist.linear.x = -0.1
    self.robot_movement_pub.publish(twist)
    rospy.sleep(1.5)
    twist.linear.x = 0
    self.robot_movement_pub.publish(twist)
    rospy.sleep(1)



def find_and_face_color(self, color):  
    print(color)
    #print("finding color")  
    hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
    #print("There's an image")
    
    #define hsv values of different colors
    bounds = {
        "blue": {
            "lower": numpy.array([85,50, 155]),
            "upper" : numpy.array([105, 255, 255])
        },
        "pink": {
            "lower": numpy.array([155,50, 155]),
            "upper" : numpy.array([170, 255, 255])
        },
        "orange": {
            "lower": numpy.array([10.93, 63.75, 150.45]),
            "upper": numpy.array([19.89, 255, 255])
        } 
    }
        
    def find_and_face_robot(self,robot_color):
        #put orange tape on the robot??
        print(robot_color)
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        bounds = {
        "orange": {
            "lower": numpy.array([10.93, 63.75, 150.45]),
            "upper": numpy.array([19.89, 255, 255])
        }
    }

    def move_robo2_to_position(self):
        #im thinking we put an AR tag on the wall and when robot get there be farish
        #we just have it rotate to face the opening of the maze so the second robot can meet it
        find_and_face_ar(self)
        back_up(self)
        #turn 90 degrees
        twist = Twist(
        linear = Vector3(),
        angular = Vector3()
    )
        twist.angular.z = 1.47/4
        rospy.Rate(2)
        for i in range(2):
            twist.angular.z = 1.47/4
        self.robot_movement_pub(twist)





    # this erases all pixels that aren't yellow
    mask = cv2.inRange(hsv, bounds[color]['lower'], bounds[color]['upper'])

    # this limits our search scope to only view a slice of the image near the ground
    h, w, d = self.image.shape

    # using moments() function, the center of the yellow pixels is determined
    M = cv2.moments(mask)
    cx = np.NAN
    cy = np.NAN
    # if there are any color pixels found
    if M['m00'] > 0:
                # center of the color pixels in the image
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        # a red circle is visualized in the debugging window to indicate
        # the center point of the yellow pixels
        cv2.circle(self.image, (cx, cy), 20, (0,0,255), -1)

    angular = 0
    #how left is the dot from the center of the image 
    rotFactor = -0.005
    if(not np.isnan(cx)):
        offCenter = cx - w/2
        #if facing robot with margin of error
        if offCenter in range(-2, 2):
            self.inFront = True
        angular = rotFactor * offCenter
    else:
        angular = 0.1
        offCenter = 0
    
    set_vel(self, 0,angular)

def find_and_face_ar(self):
    h, w, d = self.image.shape
    #turn the image into a grayscale
    gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected_points = cv2.aruco.detectMarkers(gray,
    self.aruco_dict)
    tag_index = np.where(ids == self.ar)[-1]
    tag_center = np.NAN
    #if found the tag
    if(tag_index.size > 0):
        tag_index = tag_index[0]
        #this gives me four by 2 array of coordinates
        these_corners = corners[tag_index][0]
        #return the x values of the tag
        tag_xs = these_corners[:,0]
        tag_center = np.average(tag_xs)
    #how left is the dot from the center of the image 
    rotFactor = -0.005

    myAngular = 0

    if(not np.isnan(tag_center)):
        offCenter = tag_center - w/2
        
        if -2 <= offCenter <= 2:
            self.inFrontAR = True
        myAngular = rotFactor * offCenter
    else:
        
        myAngular = 0.1
        offCenter = 0
    
    set_vel(self, 0, myAngular)
