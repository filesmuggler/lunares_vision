import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo


class IntelCam:
    def __init__(self, modes: list):
        self.pipeline = set_
