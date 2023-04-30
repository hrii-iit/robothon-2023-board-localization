#! /usr/bin/env python3

import rospy
import actionlib

import hrii_trajectory_planner.msg

class TrajectoryClientHelper:
    def __init__(self):
        print("TrajectoryClientHelper")

    def init(self, robot_id, trajectory_handler_node_name):
        self.robot_id_ = robot_id
        self.traj_action_name_ = '/'+self.robot_id_+'/'+trajectory_handler_node_name+'/execute_trajectory'
        self.traj_client_ = actionlib.SimpleActionClient(self.traj_action_name_, hrii_trajectory_planner.msg.TrajectoryHandlerAction)
        
        rospy.loginfo("Wait for trajectory handler "+rospy.resolve_name(self.traj_action_name_)+" action server...")
        self.traj_client_.wait_for_server()
        rospy.loginfo("Trajectory handler available.")

    def move_to_target_pose_and_wait(self, pose, execution_time):
        goal = hrii_trajectory_planner.msg.TrajectoryHandlerGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.robot_id_
        goal.waypoint_poses.append(pose)
        print(pose)
        goal.execution_time = execution_time
        goal.sampling_time = 0.001
        goal.use_actual_pose_as_starting_pose = True
        self.traj_client_.send_goal(goal)
        self.traj_client_.wait_for_result()
        print(self.traj_client_.get_result())
        return True

