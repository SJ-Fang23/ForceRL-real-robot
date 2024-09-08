#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool


class GraspGoalPublisher:
    def __init__(self) -> None:
        rospy.init_node('send_pose', anonymous=True)
        self.pub = rospy.Publisher('/goal_position', Pose, queue_size=1)
        self.grasp_pub = rospy.Publisher('/do_grasp', Bool, queue_size=1)
        self.sub_feedback = rospy.Subscriber('/execution_feedback', Bool, self.feedback_cb)
        self.pose = Pose()
        self.grap_raw = rospy.get_param('world_frame_grasp')
        self.grasp_poses = []

        prepare_grasp_pose = Pose()
        prepare_grasp_pose.position.x = self.grap_raw[0]
        prepare_grasp_pose.position.y = self.grap_raw[1]
        prepare_grasp_pose.position.z = self.grap_raw[2] + 0.25
        self.grasp_poses.append(prepare_grasp_pose)

        final_grasp_pose = Pose()
        final_grasp_pose.position.x = self.grap_raw[0]
        final_grasp_pose.position.y = self.grap_raw[1]
        final_grasp_pose.position.z = self.grap_raw[2] + 0.2
        self.grasp_poses.append(final_grasp_pose)

        self.grasp_pose_index = 0
        self.gripper_closed = False

        self.pose = self.grasp_poses[self.grasp_pose_index]

    def send_pose(self, pose):
        input(f"Press Enter to send the action {self.grasp_pose_index}...")
        self.pub.publish(pose)
    
    def close_gripper(self):
        input("Press Enter to close the gripper...")
        self.grasp_pub.publish(Bool(data=True))
        self.gripper_closed = True

    
    def feedback_cb(self, feedback):
        print("Feedback received: ", feedback)
        if feedback:
            print("Executing next grasp pose")
            self.grasp_pose_index += 1
            if self.grasp_pose_index < len(self.grasp_poses):
                self.pose = self.grasp_poses[self.grasp_pose_index]
                self.send_pose(self.pose)
            elif self.grasp_pose_index == len(self.grasp_poses) and not self.gripper_closed:
                print("Grasp poses executed, closing gripper")
                self.close_gripper()
            else:
                print("Grasp poses executed and gripper closed")
                rospy.signal_shutdown("Grasp poses executed and gripper closed")


    def start(self):
        rospy.loginfo("Starting grasp goal publisher")
        self.send_pose(self.pose)
        rospy.spin()

if __name__ == '__main__':
    grasp_goal_publisher = GraspGoalPublisher()
    grasp_goal_publisher.start()
    # send_pose()
    # rospy.spin()