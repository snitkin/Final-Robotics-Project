#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
import os
import pandas as pd
import numpy as np

import moveit_commander

import math

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

kp = 0.05

class Final:

    def __init__(self):
        
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

    # sets the robot linear and angular velocity
    # uses self.twist
    def set_vel(self, linear, angular):

        self.twist.linear.x = linear
        self.twist.angular.z = angular
        self.robot_movement_pub.publish(self.twist)

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

    def find_and_face_color(self):  
        #print("finding color")  
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        #print("There's an image")
        
        #define hsv values of different colors
        bounds = {
            "pink": {
                "lower": numpy.array([155,50, 155]),
                "upper" : numpy.array([170, 255, 255])
            }                 
        }
        
        # this erases all pixels that aren't yellow
        mask = cv2.inRange(hsv, bounds[self.color]['lower'], bounds[self.color]['upper'])

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
        
        self.set_vel(0,angular)

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
        
        self.set_vel(0, myAngular)


    # Uses proportional control to drive to a target based on self.scan 
    # looks at 10 degrees in either direction
    def drive_to_target(self):

        self.set_movement_arm()

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

            self.set_vel(0.05, angular)
            rospy.sleep(0.1)  

        print("done with while")

        self.set_vel(0,0)
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
        self.set_gripper(gripper_joint_goal)

        goal = [0,5,0,0]
        self.set_arm(goal)
        print("gripper in position")
        rospy.sleep(2)

    # Grips and lifts an object above the Lidar
    def grip_and_lift(self):
        print("lifting")
        gripper_joint_goal = [-0.007, -0.007]
        self.set_gripper(gripper_joint_goal)
        goal = [0,-50,0,0]
        self.set_arm(goal)
        rospy.sleep(2)
        print("lifted")
    
    # Lowers and releases object
    def lower(self):
        print("putting")
        goal = [0,0,0,0]
        self.set_arm(goal)

        gripper_joint_goal = [0.01, 0.01]
        self.set_gripper(gripper_joint_goal)
        self.time_to_drop = False
        print("put")

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

    def do_actions(self):
        return True

    def run(self):
            rospy.spin()
            
if __name__ == '__main__':

    follower = Final()
    follower.run()
