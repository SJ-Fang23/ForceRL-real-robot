# !/usr/bin/env python3


import rospy
import tf2_ros
import tf
import numpy as np
import os
import yaml
from visualization_msgs.msg import Marker


class GetRobotFrameGrasp:
    def __init__(self, cf_grasp_file_path):
        self.cf_grasp_file_path = cf_grasp_file_path
        self.cf_grasp = np.load(cf_grasp_file_path, allow_pickle=True)
        # npz to dict
        self.cf_grasp = self.cf_grasp["data"].item()
        # print(self.cf_grasp.files)
        # self.cf_grasp = self.cf_grasp['grasp']
        # print(self.cf_grasp['proposals']) 
        # sort by score
        self.sorted_grasps = sorted(self.cf_grasp['proposals'], key=lambda x: x[2], reverse=True)
        print(self.sorted_grasps)
        rospy.init_node('get_world_frame_grasp')

    def transform_to_robot_frame(self):
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            # camera_depth_optical_frame
            listener.waitForTransform("world", "camera_depth_optical_frame", rospy.Time(0), rospy.Duration(10.0))
            (trans, rot) = listener.lookupTransform("world", "camera_depth_optical_frame", rospy.Time(0))
            trans_matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans),
                                                                     tf.transformations.quaternion_matrix(rot))
            # print(trans_matrix)
            grasp = self.sorted_grasps[0][0]
            # make homogeneous
            grasp = np.append(grasp, 1)
            # do transformation
            grasp = np.dot(trans_matrix, grasp)
            # set rosparam
            # rospy.set_param('robot_frame_grasp', list(grasp))
            print(grasp)
            # shutdown
            rate.sleep()
            
            break
        # visualize
        # seng grasp to rosparam
        rospy.set_param('world_frame_grasp', grasp.tolist())
        self.visualize_grasp(grasp[:3])
    
    def visualize_grasp(self, point):
        # rospy.init_node('get_robot_frame_grasp', anonymous=True)
        pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        # set marker
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "basic_shapes"
        marker.id = 0
        marker.type = Marker.ARROW
        # marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        while not rospy.is_shutdown():
            # print("publishing marker")
            pub.publish(marker)
            rospy.sleep(1.0)
            # break


if __name__ == '__main__':
    # rospy.init_node('get_robot_frame_grasp')
    project_path = os.path.dirname(os.path.dirname(os.path.dirname((os.path.dirname(os.path.abspath(__file__))))))
    grasp_file_path = os.path.join(project_path, "real_small_drawer.npz")
    # cf_grasp_file_path = rospy.get_param('~cf_grasp_file_path')
    grasp_getter = GetRobotFrameGrasp(grasp_file_path)
    grasp_getter.transform_to_robot_frame()

    # rospy.spin()