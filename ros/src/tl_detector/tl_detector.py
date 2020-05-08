#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane, MaximumVelocity
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
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
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
    

    def get_closest_waypoint(self, pose_x, pose_y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx = self.waypoint_tree.query([pose_x, pose_y], 1)[1]
        
        return closest_idx
    

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

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        #print('running the Clf')
        return self.light_classifier.get_classification(cv_image)

    
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
         
        closest_light = None
        line_wp_idx = None
        light_in_range = False
                
        # for debuging only
        # state = self.get_light_state(None)
        # return line_wp_idx, state

        #light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            
            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                # Get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # Find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                # Allow a stop up to a short distance into the intersection to accommodate late light
                #  state classifications or stale yellow to red light changes in the simulator.
                if d >= self.tl_post_stop_line_view_thresh and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx
        
            # Classify once every 10 images when far away from a traffic light, and increase this rate
            #  when near.
            if diff > DIFF_THRESHOLD:
                self.image_count_thres = 10
            else:
                light_in_range = True
                if diff < 20:
                    # Option to increase image classification frequency when the vehicle is close to the
                    #  traffic light. This parameter may be best utilized on accurate, fast classifiers,
                    #  if the system can handle the extra compute overhead.
                    self.image_count_thres = 4
                else:
                    self.image_count_thres = 4
                               
            
            print('closest light:', diff, self.image_count_thres, 'working state:', self.working_state)
        
        if closest_light and light_in_range:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state, diff
        
        return -1, TrafficLight.UNKNOWN, 1000


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
