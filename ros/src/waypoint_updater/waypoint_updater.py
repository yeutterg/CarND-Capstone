#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Bool, Int32, Float32
import waypoint_helper
import math
from scipy.spatial import KDTree
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.current_pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.base_waypoints_num = None
        self.current_waypoint_index = None
        self.stop_waypoint_index = None
        
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/obstacle_waypoints', PoseStamped, self.obstacle_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.current_waypoint_pub = rospy.Publisher('current_waypoint', Int32, queue_size=1)

        self.loop()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, msg):
        self.base_waypoints = msg.waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in msg.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.stop_waypoint_index = int(msg.data)

    def obstacle_cb(self, msg):
        pass

    def loop(self):
        rate = rospy.Rate(50)  # slowed from 10Hz to 1Hz for performance
        while not rospy.is_shutdown():
            self.update_waypoints()
            rate.sleep()

    def get_current_waypoint_index(self):
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]
        cl_vect = np.array(closest_coord)
        pre_vect = np.array(prev_coord)
        pos_vec = np.array([x,y])
        val = np.dot(cl_vect-prev_coord, pos_vec-cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx
    
    def should_update_waypoints(self):
        return self.get_basic_waypoints() and self.current_pose

    def get_basic_waypoints(self):
        return self.base_waypoints

    def get_final_waypoints(self):
        return self.base_waypoints[self.current_waypoint_index:self.current_waypoint_index+LOOKAHEAD_WPS]
    
    def decelerate_car(self, final_waypoints, current_waypoint_index, stop_waypoint_index):
        new_final_waypoints = []
        for i, wp in enumerate(final_waypoints):
            stop_idx = max(stop_waypoint_index - current_waypoint_index - 2, 0)
            dist = waypoint_helper.linear_arc_distance(final_waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            p = Waypoint()
            p.pose = wp.pose
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            new_final_waypoints.append(p)
        return new_final_waypoints

    def set_final_waypoints_velocity(self):
        farthest_index = self.current_waypoint_index + LOOKAHEAD_WPS
        if not self.stop_waypoint_index or self.stop_waypoint_index == -1 or self.stop_waypoint_index >= farthest_index:
            return self.get_final_waypoints()
        else:
            return self.decelerate_car(self.get_final_waypoints(), self.current_waypoint_index, self.stop_waypoint_index)

    def update_waypoints(self):
        if not self.should_update_waypoints():
            return

        # get closet waypoint index to our car
        closet_waypoint_index = self.get_current_waypoint_index()

        self.current_waypoint_index = closet_waypoint_index
        self.current_waypoint_pub.publish(Int32(self.current_waypoint_index))

        lane = Lane()
        lane.header.stamp = rospy.Time.now()

        # set the velocity based on traffic lights and obstacles
        final_waypoints = self.set_final_waypoints_velocity()

        lane.waypoints = final_waypoints

        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
