#!/usr/bin/env python  

"""
Implements a very simplistic simulator for a single agent in a grid-based
world.  The world model is obtained as an occupancy grid from the map server.
There is no explicit visualization other than the fact that this node publishes
a marker for RViz.
"""

import rospy
from utils import apply_move
from utils import apply_move_with_noise

from a5_value_it.msg import Position
from a5_value_it.msg import Move
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

class GridSim:
    def __init__(self):
        self.got_map = False

        rospy.init_node('gridsim')

        # Subscribe to the map
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Subscribe to the cmd_move topic
        rospy.Subscriber('/cmd_move', Move, self.cmd_move_callback)

        # Create a publisher to output the agent's current position
        self.pos_publisher = rospy.Publisher('/pos', Position, queue_size=1)

        # Publish a marker showing the agent's position
        self.marker_publisher = rospy.Publisher('/pos_marker', Marker, \
                                                queue_size=1)

    def map_callback(self, msg):
        self.occupancy_grid = msg

        # Initialize the agent's position
        # Set to middle of map (better to initialize to a random free location).
        self.pos = Position()
        self.pos.x = self.occupancy_grid.info.width / 2
        self.pos.y = self.occupancy_grid.info.height / 2

        self.got_map = True

    def cmd_move_callback(self, move):
        if not self.got_map:
            return

        width = self.occupancy_grid.info.width
        current_cell = self.occupancy_grid.data[width * self.pos.y + self.pos.x]

        new_x, new_y = 0, 0
        if current_cell == 0:
            # The agent is on solid ground.
            new_x, new_y = apply_move(self.pos, move)
        else:
            # The agent is on an icey surface.
            new_x, new_y = apply_move_with_noise(self.pos, move)

        if self.occupancy_grid.data[width * new_y + new_x] != 100:
            # The new position is not an obstacle.
            self.pos.x = new_x
            self.pos.y = new_y

    def publish_marker(self):

        marker = Marker()

        # Initialize basic fields of the marker
        shape = Marker.CUBE
        marker.header.frame_id = '/map'
        marker.header.stamp = rospy.Time() #rospy.Time.now()
        marker.ns = 'gridsim'
        marker.id = 0
        marker.type = shape
        marker.action = Marker.ADD

        marker.pose.position.x = self.pos.x + 0.5
        marker.pose.position.y = self.pos.y + 0.5
        marker.pose.position.z = 1.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.75

        self.marker_publisher.publish(marker)

    def loop(self):
        # This is the big loop that comprises the simulation.
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # If there is no map available, we just sit and wait.
            if self.got_map:
                self.pos_publisher.publish(self.pos)
                self.publish_marker()
                #rospy.loginfo("gridsim: Main loop iteration.")
            r.sleep()

if __name__ == '__main__':
    gridsim = GridSim()
    gridsim.loop()
