#!/usr/bin/env python3

from tkinter import N
import rospy
import numpy as np
import os
import pandas as pd

from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from likelihood_field import LikelihoodField
from sensor_msgs.msg import LaserScan

# Ros message import template
# from q_learning_project.msg import QMatrix

#TODO define buffer as robot size in meter

class mapNode(object):
    def __init__(self):
        self.gn = np.nan
        self.hn = np.nan
        self.fn = np.nan
        self.parent_i = np.nan
        self.parent_j = np.nan
        self.valid = False


class Algo(object):
    def __init__(self):
        '''# grab the map from the map server
        print("hi")
        rospy.wait_for_service("static_map")
        print("bye")
        static_map = rospy.ServiceProxy("static_map", GetMap)
        self.map = static_map().map'''

        # Initialize this node
        rospy.init_node("algo")

        
        # test values
        self.start_x = 4 
        self.start_y = 6 
        self.goal_x = 10
        self.goal_y = 10
        self.width = 10
        self.height = 6
        
        #set topic names
        self.map_topic = "map"
        self.scan_topic = "scan"

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # inialize our map and likelihood field
        self.map = OccupancyGrid()
        print("map intialized")
        
        self.likelihood_field = LikelihoodField()
        print("map and field intialized")
        


        #intialize start node 
        #intialize goal node
        self.intialize_node_values()
        self.a_star()

        

    
    def get_map(self, data):

        self.map = data

    def robot_scan_received(self, data):
        return

    def intialize_node_values(self):
        print("started")
        buffer = .18
        init_node = mapNode
        self.node_values = np.full((self.height,self.width),init_node)
        for i in range(self.map.info.height):
            for j in range(self.map.info.width):
                thisNode = mapNode()
                #what does this return if pixel is not on occupancy grid
                dist = self.likelihood_field.get_closest_obstacle_distance(i,j)
                if dist > buffer:
                    thisNode.valid = True
                    #intialize hn with manhattan distance to goal
                    #TODO make sure goal is intialized with x and y
                    thisNode.hn =  (abs(i - self.goal_x) + 
                        abs(j - self.goal_y))          
                self.node_values[i][j] = mapNode()
        print("node values initialized")

    def test_valid(self):
        valid = np.full((self.height,self.width),0)
        valid[1][2] = -1
        for i in range(2,7):
            valid[2][i] = -1
        self.valid = valid
        print(self.valid)

    def test_init_nodes(self,x,y):
        self.test_valid()
        
        for i in self.map.info.height:
            for j in self.map.info.width:
                thisNode = mapNode()
                if self.valid[i][j] == 0:
                    thisNode.valid = True
                self.node_values[i][j] = mapNode()
        print(self.node_values)

    def set_new_node(self, x, y):
        if x == self.goal_x and y == self.goal_y:
            self.found = True
        if self.node_values[x][y].valid:
                
            #calculate values for new node
            new_node = mapNode()
            new_node.parent_i = self.parent_i
            new_node.parent_j = self.parent_j
            #distance from previous parent plus change
            new_node.gn = self.node_values[parent_i][parent_j].gn + change
            #distance to goal already calculated (can do calculation here if needed)
            new_node.hn = self.node_values[x][y].hn
            new_node.fn = new_node.hn + new_node.gn
            new_node.valid = True
            if (np.isnan(self.node_values[x][y].fn) or 
                (self.node_values[x][y].fn > new_node.fn)):
                #compare values of new node
                self.node_values[x][y] = new_node
                self.open[x][y] = True
    
      
            

    def a_star(self):
        max_width = self.map.info.width
        max_height = self.map.info.height
        self.open = np.full((max_height, max_width), False)
        self.closed = np.full((max_height, max_width), False)

        #set gn of start to be 0
        self.node_values[self.start_x][self.start_y].gn = 0
        self.node_values[self.start_x][self.start_y].fn = (
                self.node_values[self.start_x][self.start_y].hn)

        #put the start node in the open list
        self.open[self.start.y][self.start.j] = True
        self.found = False

        #end conditions for a star, found goal or open list empty
        while not self.found and self.open.any():
            min_f = np.inf
            parent_i = 0
            parent_j = 0
            open_list = np.transpose(np.nonzero(self.open))
            for coords in open_list:
                #access node info from open list
                node = self.node_values[coords[0]][coords[1]]
                #set parent node to first node with min f
                if node.fn < min_f:
                    min_f = node.fn
                    parent_i = coords[0]
                    parent_j = coords[1]
            self.parent_i = parent_i
            self.parent_j = parent_j
            #remove parent from open list
            self.open[parent_i][parent_j] = False

            #add parent to closed list
            self.closed[parent_i][parent_j] = True

            #find and generate the 4 children
            change = 1
            #north child, x the same, y = y-1
            x = parent_i
            y = parent_j - change
            self.set_new_node(x, y)
            #south child, x the same y = y+1
            x = parent_i
            y = parent_j + change
            self.set_new_node(x, y)
            #west child
            x = parent_i - change
            y = parent_j 
            self.set_new_node(x, y)
            #east child
            x = parent_i + change
            y = parent_j
            self.set_new_node(x, y)
        self.find_path()
    
    def find_path(self):
        path = []
        old_x = self.goal_x
        old_y = self.goal_y
        while old_x != self.start_x or old_y != self.start_y:
            x = self.node_values[old_x][old_y].parent_i
            y = self.node_values[old_x][old_y].parent_j
            coord = [x,y]
            print(coord)
            path.insert(0,coord)
            old_x = x
            old_y = y




if __name__ == "__main__":
    node = Algo()
