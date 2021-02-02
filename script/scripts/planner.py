#!/usr/bin/env python  

"""
Implements probabilistic planning based on value iteration.  Subscribes to the
/goal_pos topic and publishes /cmd_move.  Visualization information is output on the /value_markers and /policy_markers topics.
"""

import rospy
import random
from utils import apply_move
from value_it import value_it
from hao_a3.msg import Position
from hao_a3.msg import Move
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped


class Planner:
    def __init__(self):
        self.update = False
        #self.policy = []
        self.pos = None
        self.goal_pos = None

        rospy.init_node('planner')

        # Subscribe to the map
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Subscribe to the /pos topic
        rospy.Subscriber('/pos', Position, self.pos_callback)

        # Subscribe to the /goal_pos topic
        rospy.Subscriber('/goal_pos', PoseStamped, self.goal_pos_callback)

        # Create a publisher to output the desired movement
        self.cmd_move_publisher = rospy.Publisher('/cmd_move', Move, \
                                                  queue_size=1)

        # Create publishers for the value function and for the policy.
        self.value_marker_publisher = rospy.Publisher('/value_markers', \
                                                      MarkerArray, latch=True,\
                                                      queue_size=1)
        self.policy_marker_publisher = rospy.Publisher('/policy_markers', \
                                                       MarkerArray, latch=True,\
                                                      queue_size=1)

    def map_callback(self, msg):
        self.occupancy_grid = msg
        self.width = self.occupancy_grid.info.width
        self.height = self.occupancy_grid.info.height

        self.values = [[0 for j in range(self.height)] \
                      for i in range(self.width)]
        self.policy = [[Move() for j in range(self.height)] \
                      for i in range(self.width)]
        self.update = True

    def pos_callback(self, msg):
        self.pos = msg
        
    def goal_pos_callback(self, msg):
        self.goal_pos = msg
        self.update = True


    def compute_plan(self):

        if self.goal_pos == None:
            return
        goal_x = int(self.goal_pos.pose.position.x)
        goal_y = int(self.goal_pos.pose.position.y)
        rospy.loginfo('goal_x: {}, goal_y: {}'.format(goal_x, goal_y))
        self.policy, path = value_it(self.occupancy_grid, goal_x, goal_y)
        self.update = False
        rospy.loginfo("planner: Plan updated")
        marker_array = MarkerArray()
        del marker_array.markers[:]
        self.policy_marker_publisher.publish(marker_array)
        for i in range(len(path)):
                marker = Marker()
                marker.header.frame_id = '/map'
                marker.header.stamp = rospy.Time()
                marker.ns = 'policy_markers'
                marker.id = i
                marker.type = Marker.ARROW
                marker.action = Marker.ADD

                dx, dy = apply_move(Position(), self.policy[path[i][0]][path[i][1]] )
                marker.points.append(Point(path[i][0] + 0.5, path[i][1] + 0.5, 0))
                marker.points.append(Point(path[i][0] + 0.5*dx + 0.5, \
                                           path[i][1] + 0.5*dy + 0.5, \
                                           0))
                marker.scale.x = 0.2
                marker.scale.y = 0.4
                marker.scale.z = 0.3
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                marker_array.markers.append(marker)
        print len(marker_array.markers)
        self.policy_marker_publisher.publish(marker_array)

    def move(self):
        if self.pos == None:
            return

        x = int(self.pos.x)
        y = int(self.pos.y)
        move = self.policy[x][y]
        self.cmd_move_publisher.publish(move)

    def loop(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.update:
                self.compute_plan()
            r.sleep()


if __name__ == '__main__':
    planner = Planner()
    planner.loop()
