import cv2
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        (rows,cols,channels) = image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(image, (50,50), 10, 255)
        cv2.imshow("traffic_light", image)
        cv2.waitKey(3)
        return TrafficLight.UNKNOWN
