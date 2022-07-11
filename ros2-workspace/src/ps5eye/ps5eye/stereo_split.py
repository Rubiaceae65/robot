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

class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  height = 1080.0
  width = 1920.0
  def camerainfo(self, publisher):
    # CameraInfo data
    self.__message_info = CameraInfo()

    self.__message_info.height = int(self.height)
    self.__message_info.width = int(self.width)
    self.__message_info.distortion_model = 'plumb_bob'
    focal_length = 0.0
    self.__message_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    self.__message_info.r = [
	1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    self.__message_info.k = [
	focal_length, 0.0, self.width / 2,
	0.0, focal_length, self.height / 2,
	0.0, 0.0, 1.0
    ]
    self.__message_info.p = [
	focal_length, 0.0, self.width / 2, 0.0,
	0.0, focal_length, self.height / 2, 0.0,
	0.0, 0.0, 1.0, 0.0
    ]
    publisher.publish(self.__message_info)

  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher')

    camera_info_url_left = 'file://' + os.path.dirname(os.path.abspath(__file__)) + '/../config/ps5eye_left.yaml'
    camera_info_url_right = 'file://' + os.path.dirname(os.path.abspath(__file__)) + '/../config/ps5eye_right.yaml'


    self._camera_info_manager_left = CameraInfoManager(self, 'ps5eye_left', namespace='/ps5eye/left')
    self._camera_info_manager_left.setURL(camera_info_url_left)
    self._camera_info_manager_left.loadCameraInfo()

    self._camera_info_manager_right = CameraInfoManager(self, 'ps5eye_right', namespace='/ps5eye/right')
    self._camera_info_manager_right.setURL(camera_info_url_right)
    self._camera_info_manager_right.loadCameraInfo()


      
    self.publeft = self.create_publisher(Image, '/ps5eye/left/image_raw', 10)
    self.pubright = self.create_publisher(Image, '/ps5eye/right/image_raw', 10)
 
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
  def stereo_publish(self,timestamp ,leftframe, rightframe):
    """
    Publish left and right images
    """


    #ros_img = Image()
    #ros_img.encoding = 'rgb8'
    #ros_img.width = img.size[0]
    #ros_img.height = img.size[1]
    #ros_img.step = ros_img.width * 3
    #ros_img.data = img.tobytes()

    leftframe.header.frame_id = 'ps5eye_left'
    rightframe.header.frame_id = 'ps5eye_right'
    leftframe.header.stamp = timestamp
    rightframe.header.stamp = timestamp
 


    left_camera_info = self._camera_info_manager_left.getCameraInfo()
    left_camera_info.header = leftframe.header
    self.infopubleft.publish(left_camera_info)

    right_camera_info = self._camera_info_manager_right.getCameraInfo()
    right_camera_info.header = rightframe.header
    self.infopubright.publish(right_camera_info)

    self.publeft.publish(leftframe)

    self.pubright.publish(rightframe)
    
 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/ps5eye/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
 
    self.image_publisher = ImagePublisher()
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    #self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    timestamp = data.header.stamp
    current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
    crop_left = current_frame[0:0+1080, 1920:1920+1920]
    crop_right = current_frame[0:0+1080, 0:0+1920]
    outleft = self.br.cv2_to_imgmsg(crop_left, encoding='bgr8')
    outright = self.br.cv2_to_imgmsg(crop_right, encoding='bgr8')

    #self.image_publisher.stereo_publish(outframe, outframe) 
    self.image_publisher.stereo_publish(timestamp, outleft, outright) 
 

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
   # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
 
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
