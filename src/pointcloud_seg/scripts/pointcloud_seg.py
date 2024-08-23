# get pointcloud from topic and do segmentation
import rospy
import os
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
import ros_numpy
import numpy as np
import cv2
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

# export the ROS_IP to get topics from remote machine
os.environ['ROS_IP'] = '10.3.13.148'
os.environ['ROS_MASTER_URI'] = 'http://10.3.4.100:11311'

# read the masks (npz)
masks = np.load('masks.npz')

microwave_mask = masks["masks"][-2]

bridge = CvBridge()

# open a opencv window to display the image



def callback(ros_data):
    print('Received image')
    
    # convert the image to opencv format
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_data, "16UC1")
    except CvBridgeError as e:
        print(e)
    # change color space
    print(cv_image.shape)
    print(cv_image.max())

    # apply the mask
    cv_image = cv_image * microwave_mask
    np.save('depth_image.npy', cv_image)
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
    # save the image
    # cv2.imwrite('depth_image.png', cv_image)
    cv_image_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
    cv_image_normalized = np.uint8(cv_image_normalized)  # Convert to 8-bit image
    
    # Convert to color image for better visualization
    color_image = cv2.applyColorMap(cv_image_normalized, cv2.COLORMAP_JET)

    # cv2.imshow('Depth Image', color_image)
    cv2.imwrite('depth_image_masked.png', color_image)
    # save the depth image data as a numpy array
    
    

    

def listener():
    rospy.init_node('pointcloud_seg', anonymous=True)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, callback)
    rospy.sleep(10)
    rospy.spin()

if __name__ == '__main__':
    listener()