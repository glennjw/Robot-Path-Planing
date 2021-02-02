#!/usr/bin/env python 


import random

from hao_a3.msg import Position
from hao_a3.msg import Move
from astar import *
import rospy
from hao_a3.srv import *
from nav_msgs.msg import OccupancyGrid


def value_it(occupancy_grid, goal_x, goal_y):
    global maze
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height

    maze = []
    for i in range(height):
        maze.append(occupancy_grid.data[i*width : i*width+width])

    print "Searching path..."

    rospy.wait_for_service('astarMain')
    try:
        global path
        path = []
        astarMain = rospy.ServiceProxy('astarMain', Path) 
        opath = astarMain(goal_x, goal_y, occupancy_grid.data, width, height) 
        opatha = opath.d
        opathb = opath.e
        for i in range(len(opatha)):
            path.append([opatha[i],opathb[i]])
        #print ('Response path is:', path)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    policies = [[Move(9) for j in range(height)] \
                 for i in range(width)]

    for i in range(len(path)-1):
        if   path[i][0] > path[i+1][0] and path[i][1] == path[i+1][1] :
            policies[path[i+1][0]][path[i+1][1]] = Move(1)
        elif path[i][0] > path[i+1][0] and path[i][1] > path[i+1][1] :
            policies[path[i+1][0]][path[i+1][1]] = Move(2)
        elif path[i][0] == path[i+1][0] and path[i][1] > path[i+1][1] :
            policies[path[i+1][0]][path[i+1][1]] = Move(3)
        elif path[i][0] < path[i+1][0] and path[i][1] > path[i+1][1] :
            policies[path[i+1][0]][path[i+1][1]] = Move(4)
        elif path[i][0] < path[i+1][0] and path[i][1] == path[i+1][1] :
            policies[path[i+1][0]][path[i+1][1]] = Move(5)
        elif path[i][0] < path[i+1][0] and path[i][1] < path[i+1][1] :
            policies[path[i+1][0]][path[i+1][1]] = Move(6)
        elif path[i][0] == path[i+1][0] and path[i][1] < path[i+1][1] :
            policies[path[i+1][0]][path[i+1][1]] = Move(7)
        elif path[i][0] > path[i+1][0] and path[i][1] < path[i+1][1] :
            policies[path[i+1][0]][path[i+1][1]] = Move(8)
        else:
            policies[path[i+1][0]][path[i+1][1]] = Move(9)


    return policies, path






