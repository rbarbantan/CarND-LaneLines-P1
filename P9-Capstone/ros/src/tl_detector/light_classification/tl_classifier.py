import rospy
import tensorflow as tf
import cv2
import numpy as np
from styx_msgs.msg import TrafficLight

SSD_GRAPH_FILE = '../../../data/ssdlite_mobilenet_v2_coco_2018_05_09/frozen_inference_graph.pb'

class TLClassifier(object):
    def __init__(self):
        self.graph = self.load_graph(SSD_GRAPH_FILE)
        self.sess = tf.Session(graph=self.graph)
        rospy.logwarn('loaded %s' % SSD_GRAPH_FILE)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # convert to RGB colorspace 
        converted = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        #with tf.Session(graph=self.graph) as sess:
        sess = self.sess
        image_tensor = sess.graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = sess.graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = sess.graph.get_tensor_by_name('detection_scores:0')
        detection_classes = sess.graph.get_tensor_by_name('detection_classes:0')

        boxes, scores, classes = sess.run([detection_boxes, detection_scores, detection_classes], feed_dict={image_tensor: np.expand_dims(converted, 0)})
        # Remove unnecessary dimensions
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)

        confidence_cutoff = 0.2
        # Filter boxes with a confidence score less than `confidence_cutoff`
        boxes, scores, classes = self.filter_boxes(confidence_cutoff, boxes, scores, classes)

        # The current box coordinates are normalized to a range between 0 and 1.
        # This converts the coordinates actual location on the image.
        (height, width, channels) = image.shape
        box_coords = self.to_image_coords(boxes, height, width)
        #self.draw_boxes(image, box_coords, classes)

        light = TrafficLight.UNKNOWN
        if len(box_coords) > 0:
            bot, left, top, right = box_coords[0, ...].astype(int)
            crop = image[bot:top, left:right]
            crop[crop<220] = 0

            h = crop.shape[0]/3
            #cv2.imshow("traffic_light", cv2.vconcat([crop[:h,:,:], crop[h:2*h,:,:], crop[2*h:,:,:]]))
            red = np.sum(crop[:h,:,:])
            yellow = np.sum(crop[h:2*h,:,:])
            green = np.sum(crop[2*h:,:,:])

            if green > red:
                light = TrafficLight.GREEN
            else:
                light = TrafficLight.RED
            #cv2.imshow("traffic_light", crop)

        #cv2.waitKey(3)
        return light

    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph
    
    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score and classes[i] == 10:  # only look at traffic lights (coco label 10)
                idxs.append(i)
        
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes

    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].
        
        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width
        
        return box_coords

    def draw_boxes(self, image, boxes, classes, thickness=4):
        """Draw bounding boxes on the image"""
        for i in range(len(boxes)):
            bot, left, top, right = boxes[i, ...]
            class_id = int(classes[i])
            color = (0, 255, 0)
            pts = np.array([[left, top], [left, bot], [right, bot], [right, top], [left, top]], np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(image, [pts], True, color, thickness)
        cv2.imshow("traffic_light", image)
        cv2.waitKey(3)
