#!/usr/bin/env python  

import rospy
import math
import value_it
from nav_msgs.msg import OccupancyGrid
from hao_a3.srv import *

class Node_Elem:

    def __init__(self, parent, x, y, dist):
        self.parent = parent
        self.x = x
        self.y = y
        self.dist = dist
        
class A_Star:
    def __init__(self, s_x, s_y, e_x, e_y, mazewidth, mazeheight):
        self.s_x = s_x
        self.s_y = s_y
        self.e_x = e_x
        self.e_y = e_y
        
        self.width = mazewidth
        self.height = mazeheight
        
        self.open = []
        self.close = []
        self.path = []
        
    def find_path(self):
        p = Node_Elem(None, self.s_x, self.s_y, 0.0)
        while True:
            self.extend_round(p)
            if not self.open:
                return
            idx, p = self.get_best()
            if self.is_target(p):
                self.make_path(p)
                return
            self.close.append(p)
            del self.open[idx]
            
    def make_path(self,p):
        while p:
            self.path.append((p.x, p.y))
            p = p.parent
        
    def is_target(self, i):
        return i.x == self.e_x and i.y == self.e_y
        
    def get_best(self):
        best = None
        bv = 100000000
        bi = -1
        for idx, i in enumerate(self.open):
            value = self.get_dist(i)
            if value < bv:
                best = i
                bv = value
                bi = idx
        return bi, best
        
    def get_dist(self, i):
        # F = G + H
        return i.dist + math.sqrt(
            (self.e_x-i.x)*(self.e_x-i.x)
            + (self.e_y-i.y)*(self.e_y-i.y))
        
    def extend_round(self, p):
        xs = (-1, 0, 1, -1, 1, -1, 0, 1)
        ys = (-1,-1,-1,  0, 0,  1, 1, 1)
        #xs = (0, -1, 1, 0)
        #ys = (-1, 0, 0, 1)
        for x, y in zip(xs, ys):
            new_x, new_y = x + p.x, y + p.y
            if not self.is_valid_coord(new_x, new_y): 
                continue
            node = Node_Elem(p, new_x, new_y, p.dist+self.get_cost(
                        p.x, p.y, new_x, new_y))
            if self.node_in_close(node):
                continue
            i = self.node_in_open(node)
            if i != -1:
                if self.open[i].dist > node.dist:
                    self.open[i].parent = p
                    self.open[i].dist = node.dist
                continue
            self.open.append(node)
            
    def get_cost(self, x1, y1, x2, y2):
        if x1 == x2 or y1 == y2:
            return 1.0
        return 1.4
        
    def node_in_close(self, node):
        for i in self.close:
            if node.x == i.x and node.y == i.y:
                return True
        return False
        
    def node_in_open(self, node):
        for i, n in enumerate(self.open):
            if node.x == n.x and node.y == n.y:
                return i
        return -1
        
    def is_valid_coord(self, x, y):
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False
        return test_map[y][x] == 0
    
    def get_searched(self):
        l = []
        for i in self.open:
            l.append((i.x, i.y))
        for i in self.close:
            l.append((i.x, i.y))
        return l
        
def print_test_map():

    for line in test_map:
        print ''.join(line)

def find_path(s_x, s_y, e_x, e_y):
    a_star = A_Star(s_x, s_y, e_x, e_y, mazewidth, mazeheight)
    a_star.find_path()
    searched = a_star.get_searched()
    af_path = a_star.path
    
    return af_path

def astarMain(req):    #(a, b, c, d ,e)(goal_x, goal_y, occupancy_grid.data, width, height) 
    global test_map
    global mazewidth
    global mazeheight
    omap = req.c
    mazewidth = req.d
    mazeheight = req.e
    Apath = []
    Apathc = []
    Apathd = []
    print ('Go to: %d %d' % (req.a, req.b) )

    test_map = []
    for i in range(mazeheight):
        test_map.append(omap[i*mazewidth : i*mazewidth+mazewidth])

    start = [len(test_map[0])/2, len(test_map)/2]      
    Apath = find_path(start[0], start[1], req.a, req.b) 
        
    for i in range(len(Apath)):
        Apathc.append(Apath[i][0])
        Apathd.append(Apath[i][1])

    return Apathc, Apathd
    return PathResponse(Apathc, Apathd) 
    
def main():
    rospy.init_node('main')
    x = rospy.Service('astarMain', Path, astarMain)
    rospy.spin()

if __name__ == "__main__":
    main()





