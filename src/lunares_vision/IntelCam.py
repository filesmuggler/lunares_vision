import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import pyrealsense2.pyrealsense2 as rs
from sensor_msgs.msg import Image, CameraInfo



class IntelCam:
    def __init__(self, user_config: dict):

        self.user_config = user_config

        self.img_width = int(self.user_config.width)
        self.img_height = int(self.user_config.height)
        self.freq = int(self.user_config.tick)
        self.serial_number = self.user_config.serial_num[0]

        if self.user_config.align:
            self.align = self.align_depth_to_color()

        self.pipeline = self.setup_camera()
        self.c_info_publisher, self.color_publisher, self.depth_publisher = self.setup_publisher()

        self.cv_bridge = CvBridge()

    def setup_camera(self):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.serial_number)
        if self.user_config.color:
            config.enable_stream(rs.stream.color, self.img_width, self.img_height,rs.format.bgr8, self.freq)
        if self.user_config.depth:
            config.enable_stream(rs.stream.depth, self.img_width, self.img_height,rs.format.z16, self.freq)

        pipeline.start(config)
        return pipeline


    def setup_publisher(self):

        color_publisher, depth_publisher, c_info_publisher = None, None, None

        if self.user_config.color:
            color_publisher = rospy.Publisher("/intelcam/rgb_image", Image, queue_size=5)
        if self.user_config.depth:
            depth_publisher = rospy.Publisher("/intelcam/depth_image", Image, queue_size=5)

        c_info_publisher = rospy.Publisher("camera_info", CameraInfo, queue_size=5)

        return c_info_publisher,color_publisher,depth_publisher

    def align_depth_to_color(self):
        return rs.align(rs.stream.color)

    def get_frame(self):
        frames =self.pipeline.wait_for_frames()
        self.color_frame = frames.get_color_frame()
        if self.user_config.align:
            aligned_frames = self.align.process(frames)
            self.aligned_depth_frame = aligned_frames.get_depth_frame()

        self.time = self.get_timestamp(frames.get_timestamp())

        if self.user_config.color:
            self.color_message = self.image_CvBridge_conversion(self.color_frame, self.cv_bridge, self.time)
        if self.user_config.depth and self.user_config.align:
            self.align_message = self.image_CvBridge_conversion(self.aligned_depth_frame, self.cv_bridge, self.time)

        self.publish_data()
    
    def get_timestamp(self,timestamp: float):
        t1 = (timestamp / 100000000)
        t2 = (t1 - int(t1)) * 100000
        time = rospy.Time(secs=int(t2), nsecs = int((t2 - int(t2))*100))

        return time

    def image_CvBridge_conversion(self,frame, bridge, time):
        image = np.asanyarray(frame.get_data())
        message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
        message.header.stamp = time
        message.header.frame_id = "intelcam"

        return message

    # publish messages
    def publish_data(self):
        # Publish color image
        self.color_publisher.publish(self.color_message)

        # Publish camera info
        #self.pub_camera_info.publish(self.camera_info)

        # Publish align depth 
        self.depth_publisher.publish(self.align_message)

    def stop_streaming(self):
        self.pipeline.stop()
    
    def run(self):
        rospy.init_node('intelcam_node', anonymous=True)
        rate = rospy.Rate(self.freq)

        while not rospy.is_shutdown():
            self.get_frame()
            rate.sleep()

        self.stop_streaming()