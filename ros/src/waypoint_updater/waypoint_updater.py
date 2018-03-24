#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Bool, Int32, Float32
import waypoint_helper

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/obstacle_waypoints', PoseStamped, self.obstacle_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.current_waypoint_pub = rospy.Publisher('current_waypoint', Int32, queue_size=1)

        self.current_pose = None
        self.base_waypoints = None
        self.base_waypoints_num = None
        self.current_waypoint_index = None
        self.stop_waypoint_index = None

        self.loop()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, msg):
        self.base_waypoints = msg.waypoints
        self.base_waypoints_num = len(self.base_waypoints)
        self.current_waypoint_index = 0

    def traffic_cb(self, msg):
        self.stop_waypoint_index = msg

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def loop(self):
        rate = rospy.Rate(10.0)  # 10Hz
        while not rospy.is_shutdown():
            self.update_waypoints()
            rate.sleep()

    def should_update_waypoints(self):
        return self.get_basic_waypoints() and self.current_pose

    def get_basic_waypoints(self):
        return self.base_waypoints

    def get_current_waypoint_index(self):
        return self.current_waypoint_index

    def get_final_waypoints(self):
        range_indices_list = waypoint_helper.get_cyclic_range_indices(0, self.get_current_waypoint_index(),
                                                                      self.base_waypoints_num, LOOKAHEAD_WPS)

        final_waypoints = []
        for index_tuple in range_indices_list:
            final_waypoints += self.get_basic_waypoints()[index_tuple[0]:index_tuple[1]]

        return final_waypoints

    def get_stop_waypoint_index(self):
        return self.get_stop_waypoint_index()

    def update_waypoints(self):
        if not self.should_update_waypoints():
            return

        # get closet waypoint index to our car
        closet_waypoint_index = waypoint_helper.get_closest_waypoint_ahead_index(self.get_basic_waypoints()
                                                                                 , self.current_pose
                                                                                 , self.current_waypoint_index
                                                                                 , self.base_waypoints_num)
        self.current_waypoint_index = closet_waypoint_index

        self.current_waypoint_pub.publish(Int32(self.current_waypoint_index))

        lane = Lane()
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = self.get_final_waypoints()

        self.final_waypoints_pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
