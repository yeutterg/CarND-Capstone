#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight, Waypoint
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import sys
import math
import tf
import yaml
import math
import waypoint_helper

STATE_COUNT_THRESHOLD = 3
MAX_TRAFFIC_LIGHT_DISTANCE_METERS = 300


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.light_classifier = TLClassifier()

        self.pose = None
        self.waypoints = None
        self.waypoints_num = None
        self.current_waypoint_index = None
        self.camera_image = None
        self.lights = []
        self.lights_dict = {}

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub7 = rospy.Subscriber('/current_waypoint', Int32, self.current_waypoint_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        self.traffic_lights_sub = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        self.waypoints_num = len(self.waypoints)

    def current_waypoint_cb(self, msg):
        self.current_waypoint_index = msg.data

    def traffic_cb(self, msg):
        self.lights = self.get_sorted_lights_with_waypoints(msg.lights)

        # TODO: only use for testing since it doesn't change
        self.traffic_lights_sub.unregister()

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_sorted_lights_with_waypoints(self, lights):
        delimiter = '#'
        for light in lights:
            key = str(light.pose.pose.position.x) + delimiter + str(light.pose.pose.position.y) \
                  + delimiter + str(light.pose.pose.position.z)
            if key in self.lights_dict:
                light.closest_stop_waypoint = self.lights_dict[key]
            else:
                light.closest_stop_waypoint = self.get_closest_stop_line_waypoint(light.pose.pose)
                self.lights_dict[key] = light.closest_stop_waypoint

        return sorted(lights, key=lambda l: l.closest_stop_waypoint)

    def get_closest_stop_line_waypoint(self, pose):
        """ the reason for all the extra processing is that we get stop line positions and traffic light positions
            in different feeds, it would be much easier if we wouldn't have to do the match and get it a priori """

        # find closest waypoint before the traffic light
        closest_waypoint_index = self.get_closest_waypoint(pose, 0, True)

        # find the closest stop line pose to waypoint
        current_stop_line_pose = self.get_closest_stop_line_pose(closest_waypoint_index)

        # find closest waypoint to stop line going backwards from the waypoint closest to traffic light
        return waypoint_helper.get_closest_waypoint_before_index(self.waypoints
                                                                 , current_stop_line_pose
                                                                 , closest_waypoint_index
                                                                 , self.waypoints_num
                                                                 , True)

    def get_closest_stop_line_pose(self, closest_waypoint_index):

        light_stop_line_positions = self.config['stop_line_positions']
        waypoint = self.waypoints[closest_waypoint_index]
        min_distance = float('inf')
        current_stop_line_position = waypoint.pose.pose.position
        for stop_line in light_stop_line_positions:
            stop_line_position = Point()
            stop_line_position.x = stop_line[0]
            stop_line_position.y = stop_line[1]
            stop_line_position.z = 0

            distance = waypoint_helper.direct_position_distance(stop_line_position, waypoint.pose.pose.position)
            if distance < min_distance:
                min_distance = distance
                current_stop_line_position = stop_line_position

        current_stop_line_pose = Pose()
        current_stop_line_pose.position = current_stop_line_position

        return current_stop_line_pose

    def get_closest_waypoint(self, pose, current_waypoint_index=0, before=False):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
            current_waypoint_index (Int32): current waypoint index
            before: (Boolean): should return waypoint before or ahead of the position

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        return self.find_closest_index(pose, self.waypoints.waypoints)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # For testing
        # return light.state

        if not self.has_image:
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        return self.light_classifier.get_classification(cv_image)

    def distance_between_pos(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.z - pos2.z) ** 2 + (pos1.y - pos2.y) ** 2)

    def get_stop_line_positions(self):
        stop_line_positions = []
        for light_position in self.config['stop_line_positions']:
            p = Waypoint()
            p.pose.pose.position.x = light_position[0]
            p.pose.pose.position.y = light_position[1]
            p.pose.pose.position.z = 0.0
            stop_line_positions.append(p)
        return stop_line_positions

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.lights or not self.pose:
            return -1, TrafficLight.UNKNOWN

        # for testing:
        state = self.get_light_state(light)

        # List of positions that correspond to the line to stop in front of for a given intersection
        #stop_line_positions = self.config['stop_line_positions']
        stop_line_position = None
        if(self.pose):
            car_index = self.find_closest_index(self.pose.pose.position, self.waypoints.waypoints)
            car_position = self.waypoints.waypoints[car_index].pose.pose.position

            # TODO find the closest visible traffic light (if one exists)

            light_index = self.find_closest_index(car_position, self.lights)
            if light_index != -1:
                light_waypoint_index = self.find_closest_index(self.lights[light_index].pose.pose.position, self.waypoints.waypoints)
                light_position = self.waypoints.waypoints[light_waypoint_index].pose.pose.position
                # rospy.loginfo("Nearest Light Position:{}".format(light_position))

                if light_waypoint_index > car_index:
                    distance_to_traffic_light = self.distance_between_pos(car_position, light_position)
                    if distance_to_traffic_light < 300:
                        light = self.lights[light_index]
                        stop_line_index = self.find_closest_index(light_position, self.get_stop_line_positions())
                        stop_line_position = self.get_stop_line_positions()[stop_line_index].pose.pose
                        stop_line_waypoint = self.find_closest_index(stop_line_position.position, self.waypoints.waypoints)

        if light and stop_line_position:
            state = self.get_light_state(light)
            return stop_line_waypoint, state

        return -1, TrafficLight.UNKNOWN

    def find_closest_index(self, pose, positions):
        index = -1
        min_distance = sys.maxint
        for i in range(len(positions)):
            distance = self.distance_between_pos(pose, positions[i].pose.pose.position)
            if distance < min_distance:
                min_distance = distance
                index = i
        return index

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
