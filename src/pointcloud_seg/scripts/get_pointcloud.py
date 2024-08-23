import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from io import BytesIO
import zlib
import os

# export the ROS_IP to get topics from remote machine
os.environ['ROS_IP'] = '10.3.13.148'
os.environ['ROS_MASTER_URI'] = 'http://10.3.4.100:11311'


class DepthImageConverter:
    def __init__(self):
        rospy.init_node('depth_image_converter', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/depth/image_rect_raw/compressedDepth', CompressedImage, self.callback)
        self.cv_image = None

    def callback(self, msg):
        # Decompress the image data
        compressed_data = msg.data
        decompressed_data = zlib.decompress(compressed_data)
        
        # Convert the decompressed data to a NumPy array
        np_arr = np.frombuffer(compressed_data, dtype=np.uint16)
        
        # Assuming the image dimensions are known or obtained
        width = msg.width
        height = msg.height
        np_arr = np_arr.reshape((height, width))
        
        self.cv_image = np_arr
        
    def display_image(self):
        if self.cv_image is not None:
            # Convert the depth image to a format OpenCV can display
            # Normalize the depth image for better visualization
            normalized_image = cv2.normalize(self.cv_image, None, 0, 255, cv2.NORM_MINMAX)
            normalized_image = np.uint8(normalized_image)
            
            # Display the image using OpenCV
            cv2.imshow('Depth Image', normalized_image)
            cv2.waitKey(1)  # Refresh display window

if __name__ == '__main__':
    converter = DepthImageConverter()
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        converter.display_image()
        rate.sleep()