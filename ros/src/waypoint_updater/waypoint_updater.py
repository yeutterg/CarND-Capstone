#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
<<<<<<< HEAD
from std_msgs.msg import Bool, Int32, Float32
from enum import Enum
import sys
import math
import copy

=======
from std_msgs.msg import Int32

import math
import sys
import tf
>>>>>>> 2e246617a996d9ef0f69cd5b861e2c90b910dc3f

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
<<<<<<< HEAD
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints to publish.
UNKNOWN = -1

class State(Enum):
    ACCELERATION = 1
    DECELERATION = 2
=======

Author: Apostolos Georgiadis <apostolos.georgiadis@gmx.ch>
Date:   23.03.2018
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
DEBUG = True
>>>>>>> 2e246617a996d9ef0f69cd5b861e2c90b910dc3f

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
<<<<<<< HEAD

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/obstacle_waypoints', PoseStamped, self.obstacle_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.max_velocity_in_mps = rospy.get_param('/waypoint_loader/velocity') / 3.6
        self.final_waypoints = None
        self.lane = None
        self.number_of_waypoints = None
        self.current_position = None
        self.current_velocity_in_mps = None
        self.state_changed = True
        self.final_waypoints = None

        self.acceleration_limit_in_mps = rospy.get_param('~accel_limit', 1.)
        self.deceleration_limit_max_in_mps = -rospy.get_param('~decel_limit', -5.)
        self.deceleration_limit_min_in_mps = min(1.0, -rospy.get_param('~decel_limit', -5.) / 2.)

        self.next_stopline_waypoint = UNKNOWN
        self.current_state = State.DECELERATION
        self.loop()

    def loop(self):
        rate = rospy.Rate(10.0) #10Hz
        while not rospy.is_shutdown():
            self.get_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        self.current_position = msg.pose.position

    def waypoints_cb(self, waypoints):
        self.lane = waypoints
        self.number_of_waypoints = len(self.lane.waypoints)

    def distance_between_pos(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2)

    def traffic_cb(self, msg):
        self.next_stopline_waypoint = msg.data
        if self.next_stopline_waypoint != UNKNOWN \
                and self.number_of_waypoints is not None:
            self.next_stopline_waypoint = (self.next_stopline_waypoint - 5 + self.number_of_waypoints) % self.number_of_waypoints
=======
	# Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
	rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
	#rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)	
	# Publisher
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
	# Current vehicle location and orientation
	self.current_pose = None
	# Base waypoints
	self.base_waypoints = None
  	# Indicates traffic light waypoint
	self.traffic_waypoint = -1
        rospy.spin()

    def pose_cb(self, msg):
        ''' Callback for /current_pose. 
	    Sets the vehicles location and orientation. 
	'''
	self.current_pose = msg.pose
	if self.base_waypoints is not None: 
        	self.update()

    def waypoints_cb(self, waypoints):
	''' 
	Callback for /base_waypoints. 
	Sets the waypoint map. This is called once. 
	''' 
        self.base_waypoints = waypoints.waypoints
	rospy.loginfo("Waypoints loaded.")

    def traffic_cb(self, msg):
        ''' 
  	Callback for /traffic_waypoint. 
	Sets the traffic waypoint. 
	''' 
        self.traffic_waypoint = msg.data
	rospy.loginfo("Traffic light at " + str(self.traffic_waypoint))
>>>>>>> 2e246617a996d9ef0f69cd5b861e2c90b910dc3f

    def obstacle_cb(self, msg):
        pass

<<<<<<< HEAD
    def current_velocity_cb(self, msg):
        self.current_velocity_in_mps = msg.twist.linear.x

    def current_state_waypoints(self, lane, waypoint_index, cv):
        j = 0
        while j < len(self.final_waypoints):
            if self.final_waypoints[j].pose.pose.position == self.lane.waypoints[waypoint_index].pose.pose.position:
                break
            j += 1

        for i in range(j, len(self.final_waypoints)):
            current_waypoint = copy.deepcopy(self.lane.waypoints[(waypoint_index + i - j) % self.number_of_waypoints])
            current_waypoint.twist.twist.linear.x = self.final_waypoints[i].twist.twist.linear.x
            lane.waypoints.append(current_waypoint)

        for i in range(len(lane.waypoints), LOOKAHEAD_WPS):  # Number of waypoints to publish
            current_waypoint = copy.deepcopy(self.lane.waypoints[(waypoint_index + i) % self.number_of_waypoints])
            current_waypoint.twist.twist.linear.x = cv
            lane.waypoints.append(current_waypoint)

    def calc_dist_x(self, waypoint_index):
        return

    def get_waypoints(self):
        if self.lane is None \
                or self.current_position is None \
                or self.current_velocity_in_mps is None:
            return

        min_distance = sys.maxint
        waypoints = self.lane.waypoints
        waypoint_index = UNKNOWN
        for i in range(self.number_of_waypoints):
            distance = self.distance_between_pos(self.current_position, waypoints[i].pose.pose.position)
            if distance < min_distance:
                min_distance = distance
                waypoint_index = i

        d_x = self.current_position.x - self.lane.waypoints[waypoint_index].pose.pose.position.x
        nx = self.lane.waypoints[(waypoint_index + 1) % self.number_of_waypoints].pose.pose.position.x - self.lane.waypoints[waypoint_index].pose.pose.position.x

        d_y = self.current_position.y - self.lane.waypoints[waypoint_index].pose.pose.position.y
        ny = self.lane.waypoints[(waypoint_index + 1) % self.number_of_waypoints].pose.pose.position.y - self.lane.waypoints[waypoint_index].pose.pose.position.y

        if d_x * nx + d_y * ny > 0.0:
            waypoint_index = (waypoint_index + 1) % self.number_of_waypoints

        lane = Lane()
        lane.header.stamp = rospy.Time.now()

        # Handle state changes
        if self.current_state == State.ACCELERATION and self.next_stopline_waypoint != UNKNOWN:
            brake_distance = self.distance_between_pos(self.current_position, self.lane.waypoints[self.next_stopline_waypoint].pose.pose.position) - 0.5
            min_brake_distance = 0.5 * self.current_velocity_in_mps ** 2 / self.deceleration_limit_max_in_mps
            max_brake_distance = 0.5 * self.current_velocity_in_mps ** 2 / self.deceleration_limit_min_in_mps
            if max_brake_distance >= brake_distance >= min_brake_distance:
                self.current_state = State.DECELERATION
                self.state_changed = True

        elif self.current_state == State.DECELERATION and self.next_stopline_waypoint == UNKNOWN:
            self.current_state = State.ACCELERATION
            self.state_changed = True

        # Handle states
        if self.current_state == State.ACCELERATION and self.state_changed:
            i = 0
            acceleration = self.acceleration_limit_in_mps
            cur_velocity = self.current_velocity_in_mps
            target_velocity = self.current_velocity_in_mps
            while target_velocity < self.max_velocity_in_mps or i < LOOKAHEAD_WPS:
                start_position = self.current_position
                end_position = self.lane.waypoints[
                    (waypoint_index + i) % self.number_of_waypoints].pose.pose.position
                distance = self.distance_between_pos(start_position, end_position)
                target_velocity = math.sqrt(cur_velocity ** 2.0 + 2.0 * acceleration * distance)
                if target_velocity > self.max_velocity_in_mps:
                    target_velocity = self.max_velocity_in_mps
                current_waypoint = copy.deepcopy(self.lane.waypoints[(waypoint_index + i) % self.number_of_waypoints])
                current_waypoint.twist.twist.linear.x = target_velocity
                lane.waypoints.append(current_waypoint)
                i += 1

        elif self.current_state == State.ACCELERATION and not self.state_changed:
            self.current_state_waypoints(lane, waypoint_index, self.max_velocity_in_mps)

        elif self.current_state == State.DECELERATION and self.state_changed:
            i = 0
            current_velocity = self.current_velocity_in_mps
            target_velocity = self.current_velocity_in_mps
            distance = self.distance_between_pos(self.current_position, self.lane.waypoints[self.next_stopline_waypoint].pose.pose.position) - 0.5
            acceleration = current_velocity ** 2.0 / (2.0 * distance)
            while target_velocity > 0.0 or i < LOOKAHEAD_WPS:
                start_position = self.current_position
                end_position = self.lane.waypoints[(waypoint_index + i) % self.number_of_waypoints].pose.pose.position
                distance = self.distance_between_pos(start_position, end_position)
                target_velocity_exp = current_velocity ** 2.0 - 2.0 * acceleration * distance
                if target_velocity_exp <= 0:
                    target_velocity = 0
                else:
                    target_velocity = math.sqrt(target_velocity_exp)
                current_waypoint = copy.deepcopy(self.lane.waypoints[(waypoint_index + i) % self.number_of_waypoints])
                current_waypoint.twist.twist.linear.x = target_velocity
                lane.waypoints.append(current_waypoint)
                i += 1

        elif self.current_state == State.DECELERATION and not self.state_changed:
            self.current_state_waypoints(lane, waypoint_index, 0)

        self.state_changed = False
        self.final_waypoints = copy.deepcopy(lane.waypoints)
        self.final_waypoints_pub.publish(lane)
=======
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_current_position(self):
	if self.current_pose is not None:
		return self.current_pose.position
	return None

    def get_current_orientation(self):
	if self.current_pose is not None:
		return self.current_pose.orientation
	return None

    def print_position(self):
	p = self.get_current_position()
        print("X: {}, Y: {}, Z: {}".format(p.x, p.y, p.z))

    def print_orientation(self):
	o = self.get_current_orientation()
        print("X: {}, Y: {}, Z: {}, W: {}".format(o.x, o.y, o.z, o.w))

    def closest_waypoint(self, pose, waypoints):
        closest_len = sys.maxint # large number
        closest_waypoint = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        for i, waypoint in enumerate(waypoints):
            dist = dl(pose.position, waypoint.pose.pose.position)
            if (dist < closest_len):
                closest_len = dist
                closest_waypoint = i
        return closest_waypoint

    def next_waypoint(self, pose, waypoints):
        closest_waypoint = self.closest_waypoint(pose, waypoints)
        map_x = waypoints[closest_waypoint].pose.pose.position.x
        map_y = waypoints[closest_waypoint].pose.pose.position.y
        heading = math.atan2((map_y - pose.position.y), (map_x - pose.position.x))
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        angle = abs(yaw - heading)        
	if angle > (math.pi / 4):        
      		closest_waypoint += 1
        return closest_waypoint

    def update(self):
	''' 
	Updates and publishes the future waypoints at /final_waypoints. 
 	'''	
	# Check if current pose  and waypoints are valid
	if self.current_pose is None or self.base_waypoints is None:
		return 	
	# Get next waypoints
	next_idx = self.next_waypoint(self.current_pose, self.base_waypoints)
	next_waypoints = self.base_waypoints[next_idx:+next_idx+LOOKAHEAD_WPS]

	# TODO star-simulators: Handle traffic lights here  

	# Create and publish Lane msg 
        lane = Lane()
	lane.header.stamp = rospy.Time.now()
	lane.waypoints = next_waypoints
	self.final_waypoints_pub.publish(lane)
>>>>>>> 2e246617a996d9ef0f69cd5b861e2c90b910dc3f

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')