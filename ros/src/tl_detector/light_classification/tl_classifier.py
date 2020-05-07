#!/usr/bin/env python

from styx_msgs.msg import TrafficLight

import cv2
import numpy as np
import tensorflow as tf
import datetime


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TLClassifier(object):
    def __init__(self, is_simulation):
        '''
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.bridge = CvBridge()
        '''

        if is_simulation:
            self.MODEL_NAME = 'light_classification/frozen-ssd_inception-simulation'
        else:
            self.MODEL_NAME = 'light_classification/frozen-ssd_inception-site'

        self.PATH_TO_FROZEN_GRAPH = self.MODEL_NAME + '/frozen_inference_graph.pb'

        #load classifier
        # Load a (frozen) Tensorflow model into memory.
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.PATH_TO_FROZEN_GRAPH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.detection_graph.get_tensor_by_name(
                'num_detections:0')
 
        self.session = tf.Session(graph=self.detection_graph)
        self.threshold = 0.5
    

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        # convert to rgb image
        #image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        image_rgb = image

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

        with self.detection_graph.as_default():
            image_expand = np.expand_dims(image_rgb, axis=0)
            start_classification_t = datetime.datetime.now()

            (boxes, scores, classes, num_detections) = self.session.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: image_expand})

            end_classification_t = datetime.datetime.now()
            elapsed_time = end_classification_t - start_classification_t

            #print("Classification took:", elapsed_time.total_seconds())

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        #print('Best class: ', classes[0])
        #print('Best score: ', scores[0])

        if scores[0] > self.threshold:
            if classes[0] == 1:
                print('Traffic Light is: GREEN')
                return TrafficLight.GREEN
            elif classes[0] == 2:
                print('Traffic Light is: RED')
                return TrafficLight.RED
            elif classes[0] == 3:
                print('Traffic Light is: YELLOW')
                return TrafficLight.YELLOW

        return TrafficLight.UNKNOWN
    
