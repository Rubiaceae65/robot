#!/usr/bin/env python3
# Basic ROS 2 program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 

# https://github.com/ros-perception/camera_info_manager_py

# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, CameraInfo
import os
#import camera_info_manager
from camera_info_manager import CameraInfoManager



class CameraInfoPublisher(Node):

    def __init__(self):
        super().__init__('ps5eye_caminfo_publisher')

        camera_info_url_left = 'file://' + os.path.dirname(os.path.abspath(__file__)) + '/../config/ps5eye_left.yaml'
        camera_info_url_right = 'file://' + os.path.dirname(os.path.abspath(__file__)) + '/../config/ps5eye_right.yaml'


        self._camera_info_manager_left = CameraInfoManager(self, 'ps5eye_left', namespace='/ps5eye/left')
        self._camera_info_manager_left.setURL(camera_info_url_left)
        self._camera_info_manager_left.loadCameraInfo()

        self._camera_info_manager_right = CameraInfoManager(self, 'ps5eye_right', namespace='/ps5eye/right')
        self._camera_info_manager_right.setURL(camera_info_url_right)
        self._camera_info_manager_right.loadCameraInfo()


        self.infopubleft = self.create_publisher(
            CameraInfo,
            '/ps5eye/left/camera_info',
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            )
        )    
        self.infopubright = self.create_publisher(
            CameraInfo,
            '/ps5eye/right/camera_info',
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            )
        ) 
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        self.subscription = self.create_subscription(
          Image, 
          '/ps5eye/right/image_raw', 
          self.image_callback_right, 
          10)
        self.subscription # prevent unused variable warning

        self.subscription = self.create_subscription(
          Image, 
          '/ps5eye/left/image_raw', 
          self.image_callback_left, 
          10)
        self.subscription # prevent unused variable warning



    def image_callback_left(self, data):
        """
        Callback function.
        """
        timestamp = data.header.stamp

        left_camera_info = self._camera_info_manager_left.getCameraInfo()
        left_camera_info.header = data.header
        left_camera_info.header.frame_id = 'ps5eye_left'
        self.infopubleft.publish(left_camera_info)

    def image_callback_right(self, data):
        """
        Callback function.
        """
        right_camera_info = self._camera_info_manager_right.getCameraInfo()
        right_camera_info.header = data.header
        right_camera_info.header.frame_id = 'ps5eye_right'
        self.infopubright.publish(right_camera_info)


    def image_callback_both(self, data):
        """
        Callback function.
        """
        timestamp = data.header.stamp

        right_camera_info = self._camera_info_manager_right.getCameraInfo()
        right_camera_info.header = data.header
        right_camera_info.header.frame_id = 'ps5eye_right'
        self.infopubright.publish(right_camera_info)

        left_camera_info = self._camera_info_manager_left.getCameraInfo()
        left_camera_info.header = data.header
        left_camera_info.header.frame_id = 'ps5eye_left'
        self.infopubleft.publish(left_camera_info)




     




    def timer_callback(self):

        left_camera_info = self._camera_info_manager_left.getCameraInfo()
        left_camera_info.header.frame_id = 'ps5eye_left'
        self.infopubleft.publish(left_camera_info)

        right_camera_info = self._camera_info_manager_right.getCameraInfo()
        left_camera_info.header.frame_id = 'ps5eye_right'
        self.infopubright.publish(right_camera_info)

  
 
def main(args=None):
    rclpy.init(args=args)

    publisher = CameraInfoPublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
