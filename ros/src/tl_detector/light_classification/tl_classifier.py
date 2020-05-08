#!/usr/bin/env python

from styx_msgs.msg import TrafficLight

import os
import sys
import cv2
import numpy as np
import tensorflow as tf
import datetime


import rospy
import time
import yaml
import label_map_util

'''
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
'''

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.safe_load(config_string)

        self.detection_graph = None
        self.num_detections = None
        self.boxes = None
        self.scores = None
        self.classes = None

        self.label_map = None
        self.category_index = None

        self.MIN_SCORE_THRESHOLD = 0.40
        self.NUM_CLASSES = 3

        # Grab path to current working directory
        CWD_PATH = os.getcwd()
        self.is_real = self.config['is_site']
        
        if self.is_real:
            MODEL_NAME = 'frozen-rcnn_inception-site'
        else:
            MODEL_NAME = 'frozen-rcnn_inception-simulation'
        
        # path to model and label
        PATH_TO_CKPT = os.path.join(CWD_PATH, 'light_classification', MODEL_NAME,'frozen_inference_graph.pb')
        PATH_TO_LABELS = os.path.join(CWD_PATH,'light_classification', MODEL_NAME,'label_map.pbtxt')

        # Load the label map
        self.label_map      = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories          = label_map_util.convert_label_map_to_categories(self.label_map, max_num_classes=self.NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        # Load the Tensorflow model into memory.
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph)
        
        sample = cv2.imread(CWD_PATH + '/sample_load.jpg')
        
        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        image_expanded = np.expand_dims(sample, axis=0)
        
        (self.boxes, self.scores, self.classes, self.num_detections) = self.sess.run([detection_boxes, detection_scores, detection_classes, num_detections],\
            feed_dict={image_tensor: image_expanded})

        rospy.loginfo("  ")
        rospy.loginfo("  ")
        rospy.loginfo("Classifier loaded successfully!")
    

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        # convert to rgb image
        #image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        #image_rgb = image

        '''
	      # image normalization
        img_yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
        # equalize the histogram of the Y channel
        img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
        # convert the YUV image back to RGB format
        image_rgb = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
        #cv2.imshow('Color input image', image)
        #cv2.imshow('Histogram equalized', image_rgb)
        #cv2.waitKey(0)
           
        image_rgb = image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_rgb, "rgb8"))
        '''

        #TODO implement light color prediction

        start_time = time.time()
       
        # Define input and output tensors (i.e. data) for the object detection classifier
        # Input tensor is the image
        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Output tensors are the detection boxes, scores, and classes
        detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        # Expand image
        image_expanded = np.expand_dims(image, axis=0)
        
        # Perform the actual detection by running the model with the image as input
        (self.boxes, self.scores, self.classes, self.num_detections) = self.sess.run([detection_boxes, detection_scores, detection_classes, num_detections],\
            feed_dict={image_tensor: image_expanded})
        
        end_time = time.time()

        if self.scores[0][0] < self.MIN_SCORE_THRESHOLD:
            light = TrafficLight.UNKNOWN
            caption = 'UNKNOWN '
        elif self.classes[0][0] == 3:
            light = TrafficLight.GREEN
            caption = 'Green: ' + str(self.scores[0][0] * 100)[:5] + '%'
        elif self.classes[0][0] == 1:
            light = TrafficLight.RED
            caption = 'Red: ' + str(self.scores[0][0] * 100)[:5] + '%'
        elif self.classes[0][0] == 2:
            light = TrafficLight.YELLOW
            caption = 'Yellow: ' + str(self.scores[0][0] * 100)[:5] + '%'
        else:
            light = TrafficLight.UNKNOWN
            caption = 'UNKNOWN '

        return light
