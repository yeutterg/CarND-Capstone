#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import sys
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

Author: Apostolos Georgiadis <apostolos.georgiadis@gmx.ch>
Date:   23.03.2018
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
DEBUG = True

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
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

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
