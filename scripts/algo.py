#!/usr/bin/env python3

from tkinter import N
import rospy
import numpy as np
import os
import pandas as pd

from likelihood_field import LikelihoodField


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
        # Initialize this node
        rospy.init_node("algo")

        #intialize start node 
        #intialize goal node
        self.intialize_node_values()
        self.a_star()


    

    def intialize_node_values(self):
        self.node_values = np.array([self.map.info.height, self.map.info.width])
        for i in self.map.info.height:
            for j in self.map.info.width:
                thisNode = mapNode()
                #what does this return if pixel is not on occupancy grid
                dist = self.likelihood_field.get_closest_obstacle_distance(i,j)
                if dist > buffer:
                    thisNode.valid = True
                    #intialize hn with manhattan distance to goal
                    #TODO make sure goal is intialized with x and y
                    thisNode.hn =  (abs(i - self.goal.x) + 
                        abs(j - self.goal.y))          
                self.node_values[i][j] = mapNode()

    def set_new_node(self, x, y):
        if x == self.goal.x and y == self.goal.y:
            self.found = True
        if self.node_values[x][y].valid:
                
            #calculate values for new node
            new_node = mapNode()
            new_node.parent_i = parent_i
            new_node.parent_j = parent_j
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
            



if __name__ == "__main__":
    node = Algo()
