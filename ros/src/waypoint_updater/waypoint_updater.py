#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Bool, Int32, Float32
import waypoint_helper
import math
from scipy.interpolate import interp1d
import copy

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


def get_waypoint_velocity(waypoint):
    return waypoint.twist.twist.linear.x


def set_waypoint_velocity(waypoints, waypoint, velocity):
    waypoints[waypoint].twist.twist.linear.x = velocity


def get_linear_velocity(velocity):
    return velocity.twist.linear.x


def has_stop_ahead(stop_waypoint_index):
    return stop_waypoint_index and stop_waypoint_index != -1


def get_optimal_acceleration_unites(target_velocity, current_velocity, optimal_acceleration_mps):
    return int(math.ceil((target_velocity - current_velocity) / optimal_acceleration_mps))


def get_optimal_deceleration_unites(current_velocity, optimal_deceleration_mps):
    return int(math.ceil(current_velocity / optimal_deceleration_mps))


def get_speed_interpolation_function(waypoints_indices_start_stop, speed_start_stop):
    return interp1d(waypoints_indices_start_stop, speed_start_stop)


def set_waypoints_speed_from_func(speed_interpolation_func, waypoints, start_index, stop_index):
    for i in range(start_index, stop_index + 1):
        set_waypoint_velocity(waypoints, i, speed_interpolation_func(i))


def set_waypoints_speed_from_target(target_speed, waypoints, start_index, stop_index):
    for i in range(start_index, stop_index + 1):
        set_waypoint_velocity(waypoints, i, target_speed)


def decelerate_car(final_waypoints, current_speed, current_waypoint_index, stop_waypoint_index,
                   optimal_deceleration_mps):
    final_waypoints_num = len(final_waypoints)
    full_deceleration_units = get_optimal_deceleration_unites(current_speed, optimal_deceleration_mps)
    waypoints_to_stop_line = stop_waypoint_index - current_waypoint_index

    # we are waiting on a red light, zero all next waypoints velocity
    if waypoints_to_stop_line <= 0:
        set_waypoints_speed_from_target(0, final_waypoints, 0, final_waypoints_num - 1)
    # need to slow down, decelerate at an optimal linear rate
    elif waypoints_to_stop_line <= full_deceleration_units:
        speed_func = get_speed_interpolation_function([-1, final_waypoints_num - 1], [current_speed, 0])
        set_waypoints_speed_from_func(speed_func, final_waypoints, 0, final_waypoints_num - 1)
    # else we have a stop line ahead, but we don't have to slow down just yet
    else:
        set_waypoints_speed_from_target(current_speed, final_waypoints, 0, final_waypoints_num - 1)


def accelerate_car(final_waypoints, target_speed, current_speed, optimal_acceleration_mps):
    final_waypoints_num = len(final_waypoints)
    full_acceleration_units = get_optimal_acceleration_unites(target_speed, current_speed, optimal_acceleration_mps)
    # already at target velocity or more, set rest of the waypoints to target velocity
    if current_speed >= target_speed:
        set_waypoints_speed_from_target(target_speed, final_waypoints, 0, final_waypoints_num - 1)
    # need to accelerate until achieving target velocity and keep it
    elif final_waypoints_num > full_acceleration_units:
        # accelerate at an optimal linear rate
        speed_func = get_speed_interpolation_function([-1, full_acceleration_units], [current_speed, target_speed])
        set_waypoints_speed_from_func(speed_func, final_waypoints, 0, full_acceleration_units)
        # keep target velocity
        set_waypoints_speed_from_target(target_speed, final_waypoints, full_acceleration_units + 1,
                                        final_waypoints_num - 1)
    # need to accelerate at an optimal rate, will not get to target velocity just yet
    else:
        speed_func = get_speed_interpolation_function([-1, full_acceleration_units], [current_speed, target_speed])
        set_waypoints_speed_from_func(speed_func, final_waypoints, 0, full_acceleration_units)


def set_final_waypoints_velocity(final_waypoints, current_speed, current_waypoint_index, stop_waypoint_index
                                 , target_speed, optimal_acceleration_mps, optimal_deceleration_mps):
    if has_stop_ahead(stop_waypoint_index):
        decelerate_car(final_waypoints, current_speed, current_waypoint_index, stop_waypoint_index,
                       optimal_deceleration_mps)
    else:
        accelerate_car(final_waypoints, target_speed, current_speed, optimal_acceleration_mps)


LOOKAHEAD_WPS = 100  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.current_pose = None
        self.base_waypoints = None
        self.base_waypoints_num = None
        self.current_waypoint_index = None
        self.stop_waypoint_index = None
        self.current_velocity = None
        self.target_speed = 0.0
        self.optimal_acceleration_mps = 5.0
        self.optimal_deceleration_mps = 2.0

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/obstacle_waypoints', PoseStamped, self.obstacle_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber("/current_velocity", TwistStamped, self.current_velocity_cb)
        rospy.Subscriber("/target_speed", Float32, self.target_speed_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.current_waypoint_pub = rospy.Publisher('current_waypoint', Int32, queue_size=1)

        self.loop()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, msg):
        self.base_waypoints = msg.waypoints
        self.base_waypoints_num = len(self.base_waypoints)
        self.current_waypoint_index = 0

    def traffic_cb(self, msg):
        self.stop_waypoint_index = int(msg.data)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def current_velocity_cb(self, velocity):
        self.current_velocity = velocity

    def target_speed_cb(self, speed):
        self.target_speed = speed.data

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

        # deep copy waypoints since they should be immutable
        return copy.deepcopy(final_waypoints)

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

        # get the next waypoints
        final_waypoints = self.get_final_waypoints()

        # set the velocity based on traffic lights and obstacles
        set_final_waypoints_velocity(final_waypoints
                                     , get_linear_velocity(self.current_velocity)
                                     , self.current_waypoint_index
                                     , self.stop_waypoint_index
                                     , self.target_speed
                                     , self.optimal_acceleration_mps
                                     , self.optimal_deceleration_mps)

        lane.waypoints = final_waypoints

        self.final_waypoints_pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
