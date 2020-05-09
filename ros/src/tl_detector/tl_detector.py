#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
#from styx_msgs.msg import Lane, MaximumVelocity
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
from std_msgs.msg import Bool, Float64
import tf
import cv2
import yaml

from scipy.spatial import KDTree
import numpy as np

STATE_COUNT_THRESHOLD = 2
DIFF_THRESHOLD = 100

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoint_tree = None
        self.waypoints_length = None
        self.waypoints_2d = None

        self.camera_image = None
        self.lights = []
        self.count = 0

        '''
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.is_simulation = not self.config["is_site"]

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.upcoming_traffic_light_state_pub = rospy.Publisher('/traffic_light_state', Int32, queue_size=1)
        self.is_sim_pub = rospy.Publisher('/simulation_status', Bool, queue_size=1, latch=True)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.is_simulation)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.working_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.image_count = -1
        self.has_image = False
        self.image_count_thres = 4
        self.stop_for_yellow = False
        self.target_velocity = 0.0
        self.tl_post_stop_line_view_thresh = 0
        '''

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        #sub4 = rospy.Subscriber('/maximum_velocity', MaximumVelocity, self.max_vel_cb)
        #sub5 = rospy.Subscriber('/displacement_threshold', Float64, self.disp_thresh_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        # We may want to use image_raw here to prevent loss of data when changing color schemes
        '''
        if self.is_simulation:
            sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
            self.tl_post_stop_line_view_thresh = -3
            rospy.logwarn("Post stop line tl view thresh: {0}".format(self.tl_post_stop_line_view_thresh))

        rospy.spin()
        '''
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string, yaml.Loader)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # Dictionary to convert light.state to String
        self.traffic_det = {4: 'UNKNOWN',
                2: 'GREEN',
                1: 'YELLOW',
                0 :'RED' }


        self.flag = False
        rospy.spin()


    def pose_cb(self, msg):
        self.pose = msg


    def waypoints_cb(self, waypoints):
        rospy.loginfo("tl_Detector: waypoints_cb")
        self.waypoints = waypoints
        self.waypoints_length = len(waypoints.waypoints)
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
    '''
    def max_vel_cb(self, msg):
        self.target_velocity = msg.velocity
        rospy.logwarn("Target Velocity (m/s): {0}".format(self.target_velocity))
    '''

    #def disp_thresh_cb(self, msg):
    #    rospy.logwarn("Displacement Threshold (m): {0}".format(msg.data))


    def traffic_cb(self, msg):
        self.lights = msg.lights


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
        self.image_count += 1
        light_wp = None

        #print("imageID:{0}".format(self.image_count))
        if self.image_count % self.image_count_thres == 0:
            self.has_image = True
            self.camera_image = msg
            light_wp, state, distance = self.process_traffic_lights()
        '''

        '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times until we start using it. Otherwise the previous stable state is
            used.
        '''

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED or state == TrafficLight.YELLOW else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        #TODO implement
        return self.waypoint_tree.query([x, y], 1)[1]


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        time_start = rospy.Time.now()
        result, caption =  self.light_classifier.get_classification(cv_image)
        time_end = rospy.Time.now()
        #debug code
        rospy.loginfo("Traffic Ground Truth :{} Predicted State :{} Time Taken :{}".format(self.traffic_det[light.state], caption, (time_end - time_start).to_sec()))



    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if not (self.pose and self.waypoints and self.waypoint_tree):
            return -1, TrafficLight.UNKNOWN


        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)


        next_stop_line = None
        next_light = None
        #TODO find the closest visible traffic light (if one exists)
        distance = self.waypoints_length   #set next stop line distance to max initially
        for i, light in enumerate(self.lights):
            stop_line = stop_line_positions[i]
            stop_line_index = self.get_closest_waypoint(stop_line[0], stop_line[1])
            index_diff = stop_line_index - car_position

            if 0 <= index_diff < distance:
                distance = index_diff
                next_stop_line = stop_line_index
                next_light = light


        if next_light:
            state = self.get_light_state(next_light)
            return next_stop_line, state
        else:
            return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
