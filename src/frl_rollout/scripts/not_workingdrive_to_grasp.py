import rospy, sys
import moveit_commander
import math
import actionlib
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
import geometry_msgs.msg

class MoveitDriveToGrasp:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_drive_to_grasp', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("right_manipulator")
        self.group.set_pose_reference_frame("right_base")
        
        # self.group.set_goal_joint_tolerance(0.01)
        self.group.set_goal_position_tolerance(0.01)

        rospy.on_shutdown(self.shutdown)
    
    def drive_to_grasp(self):
        # get grasp from rosparam
        grasp = rospy.get_param('world_frame_grasp')

        # set target pose
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = grasp[0]
        target_pose.position.y = grasp[1] - 0.05
        target_pose.position.z = grasp[2] - 0.05

        # no rotation for now
        # target_pose.orientation.w = 1.0

        # set target
        self.group.set_pose_target(target_pose)



        # ask for confirmation
        input("Press Enter to execute the go...")
        # go
        self.group.go(wait=True)

        # stop  
        self.group.stop()
        self.group.clear_pose_targets()
    
    def shutdown(self):
        moveit_commander.roscpp_shutdown()

class MoveToGraspCilent:
    def __init__(self) -> None:
        rospy.init_node('move_to_grasp_client', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        print("Waiting for server")
        # wait for server
        self.client.wait_for_server()
        print("Server is ready")
    def drive_to_grasp(self):
                # get grasp from rosparam
        grasp = rospy.get_param('world_frame_grasp')


        # set goal
        goal = MoveGroupGoal()

        goal.request.group_name = "right_manipulator"
        goal.request.pose_goal.x = grasp[0]
        goal.request.pose_goal.y = grasp[1]
        goal.request.pose_goal.z = grasp[2] + 0.1

        # ask for confirmation
        input("Press Enter to send the action...")

        # send goal
        self.client.send_goal(goal)
        # wait for result
        self.client.wait_for_result()
        print(self.client.get_result())


if __name__ == '__main__':
    move_to_grasp = MoveToGraspCilent()
    move_to_grasp.drive_to_grasp()





