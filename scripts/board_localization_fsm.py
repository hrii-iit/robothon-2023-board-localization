#! /usr/bin/env python3

import rospy
import tf2_ros
from dynamic_reconfigure.server import Server
from hrii_board_localization.cfg import board_localization_paramConfig

from geometry_msgs.msg import Quaternion, PoseStamped, Pose, TransformStamped, Transform
from sensor_msgs.msg import Image, CameraInfo
from hrii_robothon_msgs.srv import BoardDetection, BoardDetectionResponse
from hrii_robot_msgs.srv import SetPose

import numpy as np
import math
import cv_bridge
from scipy.spatial.transform import Rotation as R
import hrii_board_localization.utils.geometry_msgs_operations as gmo

import board_localization as bl
import visual_servoing as vs
from hrii_board_localization.utils.trajectoy_client_helper import TrajectoryClientHelper

class BoardLocalizationFSM:
    def __init__(self, image_raw_topic_name, reference_frame):
        rospy.loginfo("BoardLocalizationFSM")
        rospy.loginfo("Image raw topic: %s", image_raw_topic_name)
        rospy.loginfo("Reference frame: %s", reference_frame)
        
        # FSM attributes
        self.new_img_received_ = False

        # OpenCV attributes
        self.cv_bridge = cv_bridge.CvBridge()

        self.board_localizator_ = bl.BoardLocalization()
        self.visual_servoing_ = vs.VisualServoing()

        self.traj_client = TrajectoryClientHelper()
        self.execution_time = 3.0
        self.close_proximity_detection_ = 0.24
        self.wait_after_moving_ = 2.0

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.camera_frame_id = ""
        self.image_raw_timeout_ = 0.3
        self.reference_frame_ = reference_frame

        self.red_button_transf_ = None
        self.blue_button_transf_ = None

        # ROS attributes
        self.activation_service_ = rospy.Service('~activate', BoardDetection, self.board_detection_activation)
        
        self.image_raw_sub_ = rospy.Subscriber(image_raw_topic_name, Image, self.image_raw_callback)
        self.dyn_rec_server_ = Server(board_localization_paramConfig, self.dyn_rec_callback)
        
    def wait_for_camera_info(self, camera_info_topic_name, msg_timeout=60):
        rospy.loginfo("Wait for %s camera info message...", camera_info_topic_name)
        camera_info = None
        try:
            camera_info = rospy.wait_for_message(camera_info_topic_name, CameraInfo, timeout=msg_timeout)
        except:
            rospy.logerr("Error waiting camera info.")
            return False
        
        if camera_info == None:
            rospy.logerr("Camera info is equal to none.")
            return False
        
        self.camera_frame_id = camera_info.header.frame_id

        self.board_localizator_.set_camera_info(camera_info)
        self.visual_servoing_.set_camera_info(camera_info)
        return True
    
    def detect(self):
        rospy.loginfo("Board detection activated.")

        if not self.wait_for_image_raw():
            return False

        color_image = self.cv_bridge.imgmsg_to_cv2(self.image_raw_, desired_encoding='bgr8')

        self.board_localizator_.localize_board(color_image)

        return True
    
    def get_board_pose1(self, robot_id):
        if self.blue_button_transf_ == None:
            rospy.logerr("No blue button transform found. Exit")
        if self.red_button_transf_ == None:
            rospy.logerr("No red button transform found. Exit")
        blue_button_z = 0.095
        blue_button_pos = np.array([self.blue_button_transf_.transform.translation.x,
                                    self.blue_button_transf_.transform.translation.y,
                                    blue_button_z])
        red_button_pos = np.array([self.red_button_transf_.transform.translation.x,
                                   self.red_button_transf_.transform.translation.y,
                                   blue_button_z])
        
        robot_link0_T_blue_button = np.identity(4)
        robot_link0_T_blue_button[0:3, 3] = blue_button_pos
        robot_link0_T_blue_button[0:3, 0] = (red_button_pos - blue_button_pos)/np.linalg.norm(red_button_pos - blue_button_pos)
        robot_link0_T_blue_button[0:3, 2] = np.array([0.0, 0.0, -1.0])
        robot_link0_T_blue_button[0:3, 1] = np.cross(robot_link0_T_blue_button[0:3, 2], robot_link0_T_blue_button[0:3, 0])

        try:
            world_T_robot_link0_transf = self.tf_buffer.lookup_transform(self.reference_frame_, robot_id+"_link0", rospy.Time(0), timeout=rospy.Duration(10))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf from "+self.reference_frame_+" to "+robot_id+"_link0. Exiting")
            return False
        world_T_robot_link0 = gmo.to_matrix(world_T_robot_link0_transf.transform)

        try:
            blue_button_link_T_board_base_link_transf = self.tf_buffer.lookup_transform("task_board_blue_button_link", "task_board_base_link", rospy.Time(0), timeout=rospy.Duration(10))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf from task_board_blue_button_link_link0 to task_board_base_link. Exiting")
            return False
        blue_button_link_T_board_base_link = gmo.to_matrix(blue_button_link_T_board_base_link_transf.transform)
        
        world_T_board_base_link = np.matmul(world_T_robot_link0, np.matmul(robot_link0_T_blue_button, blue_button_link_T_board_base_link))
        print("world_T_board_base_link")
        print(world_T_board_base_link)
        return world_T_board_base_link

    def get_board_pose2(self, robot_id):
        # self.blue_button_transf_ = TransformStamped()
        # self.blue_button_transf_.transform.translation.x = 0.3938
        # self.blue_button_transf_.transform.translation.y = -0.1991
        # self.blue_button_transf_.transform.rotation.w = 1.0

        # self.red_hole_transf_ = TransformStamped()
        # self.red_hole_transf_.transform.translation.x = 0.416
        # self.red_hole_transf_.transform.translation.y = -0.258
        # self.red_hole_transf_.transform.rotation.w = 1.0

        # world_T_board_pose = Transform()
        # world_T_board_pose.translation.x = 0.0555
        # world_T_board_pose.translation.y = -0.0275

        if self.blue_button_transf_ == None:
            rospy.logerr("No blue button transform found. Exit")
        if self.red_hole_transf_ == None:
            rospy.logerr("No red hole transform found. Exit")
        blue_button_z = 0.095
        blue_button_pos = np.array([self.blue_button_transf_.transform.translation.x,
                                    self.blue_button_transf_.transform.translation.y,
                                    blue_button_z])
        red_hole_pos = np.array([self.red_hole_transf_.transform.translation.x,
                                   self.red_hole_transf_.transform.translation.y,
                                   blue_button_z])
        
        red_hole_blue_button_axis = (blue_button_pos-red_hole_pos)/np.linalg.norm(blue_button_pos-red_hole_pos)
        print("red_hole_blue_button_axis")
        print(red_hole_blue_button_axis)
        x_axis = np.array([1,0,0])
        theta = math.acos(np.dot(x_axis, red_hole_blue_button_axis))
        print("Original theta")
        print(theta)
        x_axis_A_new_axis = 1.9907
        # Add offset for each quadrants
        if red_hole_blue_button_axis[0] > 0 and red_hole_blue_button_axis[1] > 0:
            print("First quadrant")
            theta = 2*3.14159 - x_axis_A_new_axis + theta
        elif red_hole_blue_button_axis[0] < 0 and red_hole_blue_button_axis[1] > 0:
            print("Fourth quadrant")
            theta = -x_axis_A_new_axis + theta
        else:
            print("Second or third quadrant")
            theta = 2*3.14159 - x_axis_A_new_axis - theta

        print("Final theta")
        print(theta)

#         if dir(1) > 0 && dir(2) > 0
#     disp("First quadrant")
#     %w_R_base = x_axis_A_new_axis + theta
#     w_R_base = 2*pi - x_axis_A_new_axis + theta
# elseif dir(1) > 0 && dir(2) < 0
#     disp("second quadrant")
#     %w_R_base = x_axis_A_new_axis - theta
#     w_R_base = 2*pi - x_axis_A_new_axis - theta
# elseif dir(1) < 0 && dir(2) < 0
#     disp("third quadrant")
#     %w_R_base = pi*2 + x_axis_A_new_axis - theta
#     w_R_base = pi*2 - x_axis_A_new_axis - theta
# elseif dir(1) < 0 && dir(2) > 0
#     disp("fourth quadrant")
#     %w_R_base = 1.1509 + theta
#     w_R_base = -x_axis_A_new_axis + theta
# end
        # world_R_base_link = np.identity(4)
        # world_R_base_link[0:3, 0] = np.array([math.cos(theta), -math.sin(theta), 0])
        # world_R_base_link[0:3, 1] = np.array([math.sin(theta), math.cos(theta), 0])
        # world_R_base_link[0:3, 2] = np.array([0, 0, 1])
        # print("world_R_base_link")
        # print(world_R_base_link)

        # red_hole_T_base_link_ext = np.identity(4)
        # red_hole_T_base_link_ext[0:3, 3] = red_hole_pos

        try:
            task_board_base_link_T_red_hole_transf = self.tf_buffer.lookup_transform("task_board_base_link", "task_board_ending_connector_hole_link", rospy.Time(0), timeout=rospy.Duration(10))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf from task_board_base_link to task_board_ending_connector_hole_link. Exiting")
            return False
        
        task_board_base_link_T_red_hole = gmo.to_matrix(task_board_base_link_T_red_hole_transf.transform)
        print("task_board_base_link_T_red_hole")
        print(task_board_base_link_T_red_hole)
        red_hole_T_task_board_base_link = np.identity(4)
        red_hole_T_task_board_base_link[0:3, 3] = -task_board_base_link_T_red_hole[0:3, 3]
        red_hole_T_task_board_base_link[0:3, 0] = np.array([1.0, 0.0, 0.0])
        red_hole_T_task_board_base_link[0:3, 1] = np.array([0.0, 1.0, 0.0])
        red_hole_T_task_board_base_link[0:3, 2] = np.array([0.0, 0.0, 1.0])
        print("red_hole_T_task_board_base_link")
        print(red_hole_T_task_board_base_link)

        # try:
        #     blue_button_link_T_board_base_link_transf = self.tf_buffer.lookup_transform("task_board_blue_button_link", "task_board_base_link", rospy.Time(0), timeout=rospy.Duration(10))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.logerr("No tf from task_board_blue_button_link_link0 to task_board_base_link. Exiting")
        #     return False
        # blue_button_link_T_board_base_link = gmo.to_matrix(blue_button_link_T_board_base_link_transf.transform)
        robot_link0_T_red_hole = np.identity(4)
        robot_link0_T_red_hole[0:3, 3] = red_hole_pos
        robot_link0_T_red_hole[0:3, 0] = np.array([math.cos(theta), math.sin(theta), 0])
        robot_link0_T_red_hole[0:3, 1] = np.array([-math.sin(theta), math.cos(theta), 0])
        robot_link0_T_red_hole[0:3, 2] = np.array([0, 0, 1])
        # world_R_base_link[0:3, 0] = np.array([math.cos(theta), -math.sin(theta), 0])
        # world_R_base_link[0:3, 1] = np.array([math.sin(theta), math.cos(theta), 0])
        # world_R_base_link[0:3, 2] = np.array([0, 0, 1])
        try:
            world_T_robot_link0_transf = self.tf_buffer.lookup_transform(self.reference_frame_, robot_id+"_link0", rospy.Time(0), timeout=rospy.Duration(10))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf from "+self.reference_frame_+" to "+robot_id+"_link0. Exiting")
            return False
        world_T_robot_link0 = gmo.to_matrix(world_T_robot_link0_transf.transform)

        world_T_board_base_link = np.matmul(world_T_robot_link0, np.matmul(robot_link0_T_red_hole, red_hole_T_task_board_base_link))
        print("world_T_robot_link0")
        print(world_T_robot_link0)
        print("robot_link0_T_red_hole")
        print(robot_link0_T_red_hole)
        print("red_hole_T_task_board_base_link")
        print(red_hole_T_task_board_base_link)
        print("world_T_board_base_link")
        print(world_T_board_base_link)

        return world_T_board_base_link
    
    def move_robot_to_visual_servoing(self, robot_id):
        
        if not self.set_robot_task_frame(robot_id, self.camera_frame_id):
            return False

        blue_button_pos = None
        while blue_button_pos == None:
        # while True:
            
            if not self.wait_for_image_raw():
                return False

            color_image = self.cv_bridge.imgmsg_to_cv2(self.image_raw_, desired_encoding='bgr8')
            blue_button_pos = self.board_localizator_.get_blue_button_3D(color_image)

        camera_T_visual_servoing_start_pose = Pose()
        camera_T_visual_servoing_start_pose.position.x = blue_button_pos.x
        camera_T_visual_servoing_start_pose.position.y = blue_button_pos.y
        camera_T_visual_servoing_start_pose.position.z = 0
        camera_T_visual_servoing_start_pose.orientation.w = 1.0

        try:
            link0_T_camera = self.tf_buffer.lookup_transform(robot_id+"_link0", self.camera_frame_id, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf from "+robot_id+"_link0 to "+self.camera_frame_id+". Exiting")
            return False
        
        link0_T_visual_servoing_start_pose = gmo.concatenate_poses(gmo.transform_to_pose(link0_T_camera.transform), camera_T_visual_servoing_start_pose)
        link0_T_visual_servoing_start_pose.position.z = self.close_proximity_detection_
        link0_T_visual_servoing_start_pose.orientation = Quaternion(x=0, y=1, z=0, w=0)
        
        self.traj_client.move_to_target_pose_and_wait(link0_T_visual_servoing_start_pose, self.execution_time)

        if not self.set_robot_task_frame(robot_id, robot_id+"_EE"):
            return False
        
        return True
    
    def move_robot_to_second_detection_pose(self, robot_id):
        
        self.traj_client.init(robot_id, "trajectory_handler")
        
        if not self.set_robot_task_frame(robot_id, self.camera_frame_id):
            return False

        

        board_pose = self.board_localizator_.get_board_pose1()
        board_pose_R_second_detection_quat = Quaternion(x=0, y=1, z=0, w=0)
        board_pose.orientation = gmo.concatenate_quat(board_pose.orientation, board_pose_R_second_detection_quat)
        
        try:
            trans = self.tf_buffer.lookup_transform(robot_id+"_link0", self.camera_frame_id, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf from "+robot_id+"_link0 to "+self.camera_frame_id+". Exiting")
            return False
        
        second_detection_pose = gmo.concatenate_poses(gmo.transform_to_pose(trans.transform), board_pose)
        board_base_link_T_second_detection_pose = Pose()
        board_base_link_T_second_detection_pose.position.y = 0.05
        board_base_link_T_second_detection_pose.position.z = 0.08
        board_base_link_T_second_detection_pose.orientation = Quaternion(x=1, y=0, z=0, w=0)
        second_detection_pose = gmo.concatenate_poses(second_detection_pose, board_base_link_T_second_detection_pose)
        second_detection_pose.orientation = Quaternion(x=0, y=1, z=0, w=0)

        self.traj_client.move_to_target_pose_and_wait(second_detection_pose, self.execution_time)

        if not self.set_robot_task_frame(robot_id, robot_id+"_EE"):
            return False
        
        return True
    
    def move_robot_over_feature(self, robot_id, feature):

        rospy.loginfo("Move robot over: "+feature)

        # Set task frame to camera frame ID
        if not self.set_robot_task_frame(robot_id, self.camera_frame_id):
            return False
        
        # Get initial robot pose
        try:
            initial_robot_trans = self.tf_buffer.lookup_transform(robot_id+"_link0", self.camera_frame_id, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf from "+robot_id+"_link0 to "+self.camera_frame_id+". Exiting")
            return False

        # Define desired pose publisher
        desired_pose_pub_ = rospy.Publisher("/"+robot_id+"/cart_hybrid_motion_force_controller/desired_tool_pose", PoseStamped, queue_size=10)
        # while desired_pose_pub_.get_num_connections() != 0:
        #     rospy.logwarn_throttle(5, "Waiting for /"+robot_id+"/cart_hybrid_motion_force_controller/desired_tool_pose")
        
        # Define the new pose
        new_desired_pose = PoseStamped()
        new_desired_pose.pose = gmo.transform_to_pose(initial_robot_trans.transform)
        new_desired_pose.header.frame_id = robot_id+"_link0"

        prev_time = rospy.Time.now().to_sec()
        position_reached = False

        while not rospy.is_shutdown() and not position_reached:

            # Wait for image
            if not self.is_image_raw_updated():
                return False
        
            color_image = self.cv_bridge.imgmsg_to_cv2(self.image_raw_, desired_encoding='bgr8')

            # Compute feature  px
            feature_px_error = self.visual_servoing_.get_feature_displacement(color_image, feature)
            if feature_px_error != None:
                dt = rospy.Time.now().to_sec() - prev_time
                
                if dt > 0:
                    prev_time = rospy.Time.now().to_sec()
                    # Compute the desired displacement
                    desired_displacement = Pose()
                    desired_displacement.position.x = -self.configs_.visual_servoing_linear_gain * feature_px_error[0] * dt
                    
                    desired_displacement.position.y = -self.configs_.visual_servoing_linear_gain * feature_px_error[1] * dt
                    desired_displacement.orientation.w = 1.0

                    # Saturate values
                    if desired_displacement.position.x > self.configs_.visual_servoing_linear_saturation:
                        desired_displacement.position.x = self.configs_.visual_servoing_linear_saturation
                    elif desired_displacement.position.x < -self.configs_.visual_servoing_linear_saturation:
                        desired_displacement.position.x = -self.configs_.visual_servoing_linear_saturation
                    if desired_displacement.position.y > self.configs_.visual_servoing_linear_saturation:
                        desired_displacement.position.y = self.configs_.visual_servoing_linear_saturation
                    elif desired_displacement.position.y < -self.configs_.visual_servoing_linear_saturation:
                        desired_displacement.position.y = -self.configs_.visual_servoing_linear_saturation
                    
                    new_desired_pose.header.stamp = rospy.Time.now()
                    new_desired_pose.pose = gmo.concatenate_poses(new_desired_pose.pose, desired_displacement)
                    
                    # Force camera frame orientation
                    new_desired_pose.pose.orientation = Quaternion(x=0, y=1, z=0, w=0)
                    desired_pose_pub_.publish(new_desired_pose)
                else:
                    rospy.logwarn("dt is equal to 0")
            else: 
                rospy.logwarn("get_feature_displacement returned None.")
            
            if self.check_position_error_2D(feature_px_error, self.configs_.visual_servoing_position_error_tolerance):
                rospy.loginfo(feature+" reached.")
                position_reached = True
        
        # Get initial robot pose
        
        return position_reached

    def calibrate_hsv_feature(self, feature):
        rospy.loginfo("Calibrate HSV.")

        while not rospy.is_shutdown():

            # Wait for image
            # if not self.is_image_raw_updated():
            #     return False
        
            color_image = self.cv_bridge.imgmsg_to_cv2(self.image_raw_, desired_encoding='bgr8')

            # Compute feature px
            self.visual_servoing_.get_feature_displacement(color_image, feature)
            
        return True

    def get_blue_button_3D(self):
        rospy.loginfo("Board detection activated.")

        if not self.wait_for_image_raw():
            return False

        color_image = self.cv_bridge.imgmsg_to_cv2(self.image_raw_, desired_encoding='bgr8')

        self.board_localizator_.get_blue_button_3D(color_image)

        return True  

    def check_position_error_2D(self, error, threshold):
        norm = math.sqrt(pow(error[0],2) + pow(error[1],2))
        rospy.loginfo_throttle(1, "Check position error: "+str(norm)+" Threshold: "+str(threshold))
        return norm < threshold

    def is_image_raw_updated(self):
        return (rospy.Time.now() - self.image_raw_.header.stamp).to_sec() < self.image_raw_timeout_
    
    def board_detection_activation(self, req):
        # Send a fake static transform to detach from the main TF tree
        fake_static_transf = TransformStamped()
        fake_static_transf.header.stamp = rospy.Time.now()
        fake_static_transf.header.frame_id = self.reference_frame_+"_that_not_exist"
        fake_static_transf.child_frame_id = "task_board_base_link"
        fake_static_transf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(fake_static_transf)

        if req.robot_id == "":
            req.robot_id = "franka_left"
            rospy.logwarn("Using default robot id: "+req.robot_id)

        self.traj_client.init(req.robot_id, "trajectory_handler")
        
        link0_T_board_detection = Pose()
        link0_T_board_detection.position.x = 0.351
        link0_T_board_detection.position.y = -0.233
        link0_T_board_detection.position.z = 0.441
        link0_T_board_detection.orientation = Quaternion(x=-0.693, y=0.706, z=-0.104, w=-0.104)
        
        self.traj_client.move_to_target_pose_and_wait(link0_T_board_detection, self.execution_time)

        rospy.loginfo("Board detected. Move to second detection pose")
        if not self.move_robot_to_visual_servoing(req.robot_id):
            rospy.logerr("Error moving the robot in the second detection pose. Exiting")
            exit()

        # # Continuous visual servoing
        # # while not rospy.is_shutdown() and board_localization_fsm.detect_blue_button():
        # #     # board_localization_fsm.detect()
        # #     loop_rate.sleep()

        # ######
        # robot_id = "franka_left"
        if not self.move_robot_over_feature(req.robot_id, "task_board_blue_button"):
            rospy.logerr("Error moving the robot over blue button pose. Exiting")
            exit()
        rospy.sleep(self.wait_after_moving_)
        try:
            self.blue_button_transf_ = self.tf_buffer.lookup_transform(req.robot_id+"_link0", self.camera_frame_id, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf from "+req.robot_id+"_link0 to "+self.camera_frame_id+". Exiting")
            exit()
        rospy.loginfo("Blue button transform:")
        rospy.loginfo(self.blue_button_transf_)

        if not self.move_robot_over_feature(req.robot_id, "task_board_red_button"):
            rospy.logerr("Error moving the robot over red button pose. Exiting")
            exit()
        rospy.sleep(self.wait_after_moving_)
        try:
            self.red_button_transf_ = self.tf_buffer.lookup_transform(req.robot_id+"_link0", self.camera_frame_id, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf from "+req.robot_id+"_link0 to "+self.camera_frame_id+". Exiting")
            exit()
        rospy.loginfo("Red button transform:")
        rospy.loginfo(self.red_button_transf_)
        
        # board_localization_fsm.get_board_pose(robot_id)

        world_T_board_base_link = self.get_board_pose1(req.robot_id)
        print("world_T_board_base_link")
        world_T_board_base_link_transf = gmo.to_transform(world_T_board_base_link)
        # model_error_calibration_params = Transform()
        # model_error_calibration_params.translation.y = 0.004
        # world_T_board_base_link_transf.translation.y -= model_error_calibration_params.translation.y
        print(world_T_board_base_link)

        static_world_T_board_base_link_transf = TransformStamped()
        static_world_T_board_base_link_transf.header.stamp = rospy.Time.now()
        static_world_T_board_base_link_transf.header.frame_id = self.reference_frame_
        static_world_T_board_base_link_transf.child_frame_id = "task_board_base_link"
        static_world_T_board_base_link_transf.transform = world_T_board_base_link_transf
        self.static_tf_broadcaster.sendTransform(static_world_T_board_base_link_transf)

        try:
            self.red_hole_transf = self.tf_buffer.lookup_transform(req.robot_id+"_link0", "task_board_ending_connector_hole_link", rospy.Time(0), timeout=rospy.Duration(10))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf from "+req.robot_id+"_link0 to task_board_ending_connector_hole_link. Exiting")
            exit()
        rospy.loginfo("task_board_ending_connector_hole_link transform:")
        rospy.loginfo(self.red_hole_transf)

        red_hole_pose = gmo.transform_to_pose(self.red_hole_transf.transform)
        red_hole_pose.position.z = self.close_proximity_detection_
        red_hole_pose.orientation = Quaternion(x=0, y=1, z=0, w=0)
        self.traj_client.move_to_target_pose_and_wait(red_hole_pose, self.execution_time)

        if not self.move_robot_over_feature(req.robot_id, "task_board_red_button"):
            rospy.logerr("Error moving the robot over red button pose. Exiting")
            exit()
        rospy.sleep(self.wait_after_moving_)
        try:
            self.red_hole_transf_ = self.tf_buffer.lookup_transform(req.robot_id+"_link0", self.camera_frame_id, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf from "+req.robot_id+"_link0 to "+self.camera_frame_id+". Exiting")
            exit()
        rospy.loginfo("New red hole transform:")
        rospy.loginfo(self.red_hole_transf_)

        world_T_board_base_link = self.get_board_pose2(req.robot_id)
        world_T_board_base_link_transf = gmo.to_transform(world_T_board_base_link)
        print("world_T_board_base_link")
        print(world_T_board_base_link)
        model_error_calibration_params = Transform()
        model_error_calibration_params.translation.y = 0.004
        world_T_board_base_link_transf.translation.y -= model_error_calibration_params.translation.y
        print(world_T_board_base_link)

        static_world_T_board_base_link_transf = TransformStamped()
        static_world_T_board_base_link_transf.header.stamp = rospy.Time.now()
        static_world_T_board_base_link_transf.header.frame_id = self.reference_frame_
        static_world_T_board_base_link_transf.child_frame_id = "task_board_base_link"
        static_world_T_board_base_link_transf.transform = world_T_board_base_link_transf
        self.static_tf_broadcaster.sendTransform(static_world_T_board_base_link_transf)

        rospy.loginfo("Base link pose estimated:")
        rospy.loginfo(static_world_T_board_base_link_transf.transform)
        res = BoardDetectionResponse()
        res.success = True
        return res

    def set_robot_task_frame(self, robot_id, task_frame):
        rospy.loginfo("Set the desired task frame to "+task_frame)

        controller_set_EE_T_task_frame_service_name = "/"+robot_id+"/cart_hybrid_motion_force_controller/set_EE_T_task_frame"
        rospy.loginfo("Wait for ROS service "+controller_set_EE_T_task_frame_service_name)
        rospy.wait_for_service(controller_set_EE_T_task_frame_service_name)
        rospy.loginfo("ROS service "+controller_set_EE_T_task_frame_service_name+" available")
        
        try:
            EE_T_camera_optical_frame = self.tf_buffer.lookup_transform(robot_id+"_EE", task_frame, rospy.Time(0))
            controller_set_EE_T_task_frame_client = rospy.ServiceProxy(controller_set_EE_T_task_frame_service_name, SetPose)
            EE_T_camera_optical_frame_pose = PoseStamped()
            EE_T_camera_optical_frame_pose.pose = gmo.transform_to_pose(EE_T_camera_optical_frame.transform)
            try:
                resp = controller_set_EE_T_task_frame_client(pose_stamped=EE_T_camera_optical_frame_pose)
                if not resp.success:
                    rospy.logerr("Error setting EE T task frame")
                    return False
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return False
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf from "+robot_id+"_link0 to "+self.camera_frame_id+". Exiting")
            return False
        
        rospy.sleep(0.5)
        return True

    def wait_for_image_raw(self):
        rospy.loginfo("Waiting for a new image.")
        self.new_img_received_ = False
        while not self.new_img_received_ and not rospy.is_shutdown():
            pass
        if self.new_img_received_ and not rospy.is_shutdown():
            rospy.loginfo("New image received.")
            return True
        else:
            rospy.loginfo("New image not received. Exiting")
            return False 
        
    def image_raw_callback(self, image):
        self.image_raw_ = image
        self.new_img_received_ = True

    def dyn_rec_callback(self, config, level):
        rospy.loginfo("Dynamic reconfigure callback")
        rospy.loginfo("px [%d:%d] py [%d:%d]", config.px_min, config.px_max, config.py_min, config.py_max)
        self.configs_ = config
        self.board_localizator_.set_configs(self.configs_)
        self.visual_servoing_.set_configs(self.configs_)
        return config

if __name__ == '__main__':
    try:
        rospy.init_node("board_localization_fsm")

        image_raw_topic_name = rospy.get_param("~image_raw_topic_name", "/camera/color/image_raw")
        camera_info_topic_name = rospy.get_param("~camera_info_topic_name", "/camera/color/camera_info")
        reference_frame = rospy.get_param("~reference_frame", "my_world")

        board_localization_fsm = BoardLocalizationFSM(image_raw_topic_name, reference_frame)
        
        board_localization_fsm.wait_for_camera_info(camera_info_topic_name)
        
        rospy.loginfo("Camera info received.")

        rospy.spin()
        
        # if not board_localization_fsm.calibrate_hsv_feature("task_board_blue_button"):
        #     rospy.logerr("Error moving the robot over red button pose. Exiting")
        #     exit()

        # Final task
        # loop_rate = rospy.Rate(4.0)
        # robot_id = "franka_left"

        # Continuous Board detection
        # while not rospy.is_shutdown() and board_localization_fsm.get_blue_button_3D():
        #     # board_localization_fsm.detect()
        #     loop_rate.sleep()
        
        # rospy.loginfo("Board detected. Move to second detection pose")
        # if not board_localization_fsm.move_robot_to_visual_servoing(robot_id):
        #     rospy.logerr("Error moving the robot in the second detection pose. Exiting")
        #     exit()

        # # Continuous visual servoing
        # # while not rospy.is_shutdown() and board_localization_fsm.detect_blue_button():
        # #     # board_localization_fsm.detect()
        # #     loop_rate.sleep()

        # ######
        # robot_id = "franka_left"
        # if not board_localization_fsm.move_robot_over_feature(robot_id, "task_board_blue_button"):
        #     rospy.logerr("Error moving the robot over blue button pose. Exiting")
        #     exit()
        
        # try:
        #     board_localization_fsm.blue_button_transf_ = board_localization_fsm.tf_buffer.lookup_transform(robot_id+"_link0", board_localization_fsm.camera_frame_id, rospy.Time(0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.logerr("No tf from "+robot_id+"_link0 to "+board_localization_fsm.camera_frame_id+". Exiting")
        #     exit()
        # rospy.loginfo("Blue button transform:")
        # rospy.loginfo(board_localization_fsm.blue_button_transf_)

        # if not board_localization_fsm.move_robot_over_feature(robot_id, "task_board_red_button"):
        #     rospy.logerr("Error moving the robot over red button pose. Exiting")
        #     exit()

        # try:
        #     board_localization_fsm.red_button_transf_ = board_localization_fsm.tf_buffer.lookup_transform(robot_id+"_link0", board_localization_fsm.camera_frame_id, rospy.Time(0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.logerr("No tf from "+robot_id+"_link0 to "+board_localization_fsm.camera_frame_id+". Exiting")
        #     exit()
        # rospy.loginfo("Red button transform:")
        # rospy.loginfo(board_localization_fsm.red_button_transf_)
        
        # board_localization_fsm.get_board_pose2(robot_id)
        
        

        # loop_rate = rospy.Rate(4.0)
        # # Continuous Board detection
        # while not rospy.is_shutdown() and board_localization_fsm.detect():
        #     # board_localization_fsm.detect()
        #     loop_rate.sleep()

        # loop_rate = rospy.Rate(4.0)
        # robot_id = "franka_left"
        # Continuous Board detection
        # while not rospy.is_shutdown() and board_localization_fsm.get_blue_button_3D():
        #     # board_localization_fsm.detect()
        #     loop_rate.sleep()
        
        # rospy.loginfo("Board detected. Move to second detection pose")
        # if not board_localization_fsm.move_robot_to_visual_servoing(robot_id):
        #     rospy.logerr("Error moving the robot in the second detection pose. Exiting")
        #     exit()

    except rospy.ROSInterruptException:
        pass