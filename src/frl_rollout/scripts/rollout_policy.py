#! /usr/bin/env python3
import torch
import stable_baselines3
import rospy
from stable_baselines3 import TD3
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import numpy as np
import time
import socket 
import pickle
import copy
# from std_srvs.srv import Empty
from frl_rollout.srv import GetEndEffectorPose

class RolloutPolicy:
    def __init__(self, model_path):

        rospy.init_node('policy_rollout', anonymous=True)
        self.initial_eef_pose = None
        # self.grasp_pub = rospy.Publisher('/do_grasp', Bool, queue_size=1)
        self.sub_feedback = rospy.Subscriber('/execution_feedback', Bool, self.feedback_cb)
        self.pose_pub = rospy.Publisher('/goal_position_delta', Pose, queue_size=1)

        # serviceproxy to get eef pose
        self.eef_client = rospy.ServiceProxy('get_eef_pose', GetEndEffectorPose)

        # connect to policy server to get action
        self.HOST = 'localhost'
        self.PORT = 1234
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.HOST, self.PORT))
        print("Policy rollout node initialized")

        rospy.on_shutdown(self.shutdown)
    
    
    def rollout(self):
        '''
        Rollout the policy
        '''
        # make array of current state
        initial_eef_position = np.array([self.initial_eef_pose.position.x, self.initial_eef_pose.position.y, self.initial_eef_pose.position.z])
        current_eef_position = np.array([self.current_eef_pose.position.x, self.current_eef_pose.position.y, self.current_eef_pose.position.z])
        eef_position_relative_to_start = np.subtract(current_eef_position, initial_eef_position)
        print("Eef position relative to start: ", eef_position_relative_to_start)

        joint_direction = np.array([-1,0,0])
        joint_direction = joint_direction / np.linalg.norm(joint_direction)

        state = np.concatenate([joint_direction, eef_position_relative_to_start])
        
        # send state to policy server to get action
        self.socket.sendall(pickle.dumps(state))
        action_raw = self.socket.recv(1024)
        # action raw: a tuple (action, None)
        action_raw = pickle.loads(action_raw)

        # scale the action to make it move 1cm
        print(type(action_raw))
        action = action_raw[0] * 0.005
        print("Predicted action: ", action)
        self.send_action(action)

    def send_action(self, action):
        # input(f"Press Enter to send the action {action}...")
        
        pose = Pose()
        pose.position.x = -action[0]
        pose.position.y = action[1]
        pose.position.z = action[2]
        pose.position.z = 0 # no z movement for now

        self.pose_pub.publish(pose)

    def start(self):
        # get initial eef pose
        self.initial_eef_pose = self.eef_client().pose
        
        self.current_eef_pose = copy.deepcopy(self.initial_eef_pose)
        self.initial_eef_pose.position.x += 0.5
        self.rollout()


    def feedback_cb(self, feedback):
        '''
        when sucess feedback is received, 
        update the current eef pose and rollout again
        '''
        print("Feedback received: ", feedback)
        if feedback:
            print("Executing next grasp pose")
            self.current_eef_pose = self.eef_client().pose
            self.rollout()
    
    def shutdown(self):
        # close socket
        self.socket.close()
        rospy.signal_shutdown("Shutting down policy")
        print('disconnected')


if __name__ == '__main__':
    policy = RolloutPolicy("prismatic_model")
    time.sleep(1)
    policy.start()
    rospy.spin()