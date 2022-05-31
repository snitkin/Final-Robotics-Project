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

    # Make sure you're running with an int afterwards (1 or 2)
    # rosrun final_robotics_project final.py <1 or 2>
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
            # Done publisher
            self.done_publisher = rospy.Publisher('done_action', 
            done_message)

        elif robot_code == 2:
            # Done subscriber
            self.done_subscriber =  rospy.Subscriber('done_action',
            done_message, self.done_callback)
            self.secured_baton = False

            #self.done_subscriber =  rospy.Subscriber('done_action',
            #done_message, self.line_callback)
            #self.objective_complete2 = False

            #Done Publisher
            self.done_publisher = rospy.Publisher('done_action', 
            done_message)

        elif robot_code == 3:
            self.done_subscriber =  rospy.Subscriber('done_action',
            done_message, self.done_callback2)

        
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
        
        self.image_init = True

        while not self.image_init:
            print("waiting for init")
        else:
            print("intialized")
            self.do_actions()

    # Done callback, for robot 2
    def done_callback(self, msg):
        # Go and find the color
        print("done callback")
        print(msg.message)
        
        # find and face orange
        find_and_face_color(self, "orange")

        # go to orange
        drive_to_target(self)


        # Pick up the baton
        gripper_joint_goal = [-0.007, -0.007]
        set_gripper(self, gripper_joint_goal)

        # Send publish
        gm = gripper_message(message="done")
        self.gripper_publisher.publish(gm)

        return

    def done_callback2(self,msg):
        print("done callback2")
        #will drive foward to finish line 
        r = rospy.Rate(2)
        print("1")
        for r in range(10):
            set_vel(self, .1,0)
            rospy.sleep(1)
            #will do a little happy dance
        print("2")
        for i in range(4):
            set_vel(self,0,.35)
            rospy.sleep(1)
            set_vel(self,0,-.4)
            rospy.sleep(1)
            set_vel(self,0,0)


        

    # Gripper callback, for robot 1
    def gripper_callback(self, msg):
        
        # If code == 1 then let go, back up 
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

        # If code == 2 - 1 has let go, lift
        else:
            print("gripper callback 2")
            grip_and_lift(self)
            self.secured_baton = True
            print("starting robot 3")     
            #after it has the baton it finds line and drops baton in a basket
        
            r = rospy.Rate(3)
            for r in range(10):
                set_vel(self,0,.9)
            print("done turning")
            rospy.sleep(1)
            
            #find_and_face_line(self, "orange")
            
        
            #how does the robot know its done with line follower?? figure that out
            
            #msg2 = False
            #while not msg2:
            #    msg2 = input("Has line follower finished")
            
            print("driving to basket")
            drive_to_basket(self)
            rospy.sleep(1)
            print("dropping in basket")
            lower(self)
            gripper_joint_goal = [0.018, 0.018]
            set_gripper(self, gripper_joint_goal)
            rospy.sleep(1)

            set_vel(self,0,0)
            #publishes that its done
            done = done_message(message="done")
            self.done_publisher.publish(done)
 

            print("im here")
        return

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

    
    def do_actions(self):

        self.reset()

        if self.robot_code == 1:

            self.min_arm_distance = 0.22

         #Goes to and picks up baton
            drive_to_target(self)

            grip_and_lift(self)
            
            a_star = False
            
            while not a_star:
                # Do the a star
                i = 1
            
            rospy.sleep(1)
            lower(self)

            done = done_message(message="done")
            self.done_publisher.publish(done)

            print("Run Robot 2 code!\n")

            s = input("Has Robot 2 grabbed the baton?")

            self.objective_complete = True
            
            # Wait for the other robot to grab the baton
            while not self.objective_complete:
                i = 1

            gripper_joint_goal = [0.018, 0.018]
            set_gripper(self, gripper_joint_goal)
            rospy.sleep(1)
            back_up(self)

            print("Robot 2 is good to lift")

        elif self.robot_code == 2:

            msg = input("Has Robot 1 lowered the baton?")

            done = done_message(message=msg)
            self.done_callback(done)

            rospy.sleep(2)
            
            # by the time we exit this loop, 2 has the baton over its head
            msg1 = False
            while not msg1:
                msg1 = input("Has robot 1 backed up?")
                
            done1 = gripper_message(message=msg1)
            self.gripper_callback(done1)

            rospy.sleep(2)

            #self.objective_complete after this
            self.objective_complete = True

            while not self.objective_complete:
                i = 1

            #print("welp")
            #msg2 = False
            while not msg2:
                msg2 = input("does robot need this")
            done2 = line_message(message=msg2)
            self.line_callback(done2)
            


            print("Run Robot 3 code!\n")
        
        elif self.robot_code == 3:

            msg2 = input("Has Robot 2 lowered the baton into the basket?")

            done = done_message(message=msg2)
            self.done_callback2(done)
            print("get here")
            #rospy.sleep(2)
            
            #not sure if i need this
            self.objective_complete = True

            while not self.objective_complete:
                i = 1
            
            
        return True

    def run(self):
        rospy.spin()
            
if __name__ == '__main__':

    # Robot code is passed in as an argument to differentiate between bots
    robot_code = sys.argv[1]
    follower = Final(int(robot_code))
    follower.run()
