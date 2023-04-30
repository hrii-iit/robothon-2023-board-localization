#! /usr/bin/env python3

import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2

from geometry_msgs.msg import Pose, Vector3, Quaternion
from hrii_board_localization.cfg import board_localization_paramConfig

class BoardLocalization:
    def __init__(self):
        print("BoardLocalization")
        self.configs_ = None
        self.camera_info_ = None
        self.board_pose_ = Pose()
        self.distance_camera_board_ = 0.48
        self.active_windows_ = list()

    def set_configs(self, configs):
        self.configs_ = configs

    def set_camera_info(self, camera_info):
        self.camera_info_ = camera_info
        self.fx_ = self.camera_info_.K[0]
        self.fy_ = self.camera_info_.K[4]
        self.cx_ = self.camera_info_.K[2]
        self.cy_ = self.camera_info_.K[5]

    def localize_board(self, image_raw):
        
        self.show_image(1, 'original image', image_raw)

        self.detect_board(image_raw)

        # self.get_board_pose()

    def detect_board(self, original_img):
        img = original_img.copy()
        preprocessed_img = self.preprocess_image(img)
        self.show_image(2, 'Preprocessed image', preprocessed_img)

        (board_min_area_rect, board_corners) = self.get_board(preprocessed_img)
        cv2.drawContours(img, [board_corners], 0, (0,0,255), 2)
        self.show_image(3, 'Board image', img)

        self.board_pose_ = self.compute_board_pose(board_min_area_rect, board_corners, img)

        # print("Center: " +str(board_min_area_rect[0][0])+" "+str(board_min_area_rect[0][1]))
        img = cv2.circle(img, (int(board_min_area_rect[0][0])+self.configs_.py_min,int(board_min_area_rect[0][1])+self.configs_.px_min),
                                    3, (255, 0, 255), 3)
        img = cv2.circle(img, (int(self.cx_), int(self.cy_)),
                                    3, (100, 100, 255), 3)
        self.show_image(10, 'Final debug image', img)
        print(self.board_pose_)
    
    def compute_board_pose(self, board, board_corners, img):
        board_pose = Pose()
        board_pose.position = self.get_board_position(board)
        
        cutted_board_img = img[min(board_corners[:, 1]):max(board_corners[:, 1]), 
                               min(board_corners[:, 0]):max(board_corners[:, 0]), :]
        self.show_image(4, "Cutted board img", cutted_board_img)

        # Apply blue mask to detect the blue button
        blue_button_hsv_lower = np.array([self.configs_.board_blue_mask_h_min, self.configs_.board_blue_mask_s_min, self.configs_.board_blue_mask_v_min])
        blue_button_hsv_upper = np.array([self.configs_.board_blue_mask_h_max, self.configs_.board_blue_mask_s_max, self.configs_.board_blue_mask_v_max])
        
        blue_mask_img = cv2.inRange(cutted_board_img, blue_button_hsv_lower, blue_button_hsv_upper)

        # Erode and dilate
        blue_mask_img = cv2.erode(blue_mask_img, None, iterations=self.configs_.erode_iterations)
        blue_mask_img = cv2.dilate(blue_mask_img, None, iterations=self.configs_.dilate_iterations)

        # self.show_image(5, "Blue mask img", blue_mask_img)

        blue_button = self.get_blue_button(blue_mask_img)
        if blue_button[0] == None:
            return None
        blue_button_orig_img = (int(blue_button[0][0]) + min(board_corners[:, 0]),
                                int(blue_button[0][1]) + min(board_corners[:, 1]))
        cv2.circle(img, blue_button_orig_img, 13, (0, 0, 255), 2)
        self.show_image(6, "Blue button img", img)

        blue_button_center3D = self.from2DTo3D(blue_button_orig_img[0], blue_button_orig_img[1], self.distance_camera_board_)

        # board_pose.rotation
        
        # yaw = board
        yaw = board[2]
        quat = R.from_matrix([[math.cos(yaw), -math.sin(yaw), 0],
                                    [math.sin(yaw), math.cos(yaw), 0],
                                    [0, 0, 1]]).as_quat()
        
        print(quat)
        board_pose.orientation.x = quat[0]
        board_pose.orientation.y = quat[1]
        board_pose.orientation.z = quat[2]
        board_pose.orientation.w = quat[3]

        return board_pose

        #         try:
        #             hsv_a[min(box[:, 1]):max(box[:, 1]), min(box[:, 0]):max(box[:, 0]), :] = 0
        #             mask_c = cv2.inRange(hsv_a, hsv_lower, hsv_upper)
        #             contours_a, _ = cv2.findContours(mask_c.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #             center_b = None
        #             c_a = max(contours_a, key=cv2.contourArea)
        #             rect_a = cv2.minAreaRect(c_a)
        #             ((x,y), (width, height), rotation) = rect_a
        #             s = f"x {np.round(x)}, y: {np.round(y)}, width: {np.round(width)}, height: {np.round(height)}, rotation: {np.round(rotation)}"
        #             M_a = cv2.moments(c_a)
        #             center_b = [int(M_a["m10"] / M_a["m00"]), int(M_a["m01"] / M_a["m00"])]
        #             center_b[0] = center_b[0] + min(box_ac[:, 0]) 
        #             center_b[1] = center_b[1] + min(box_ac[:, 1])
        #             cv2.circle(orginal_img, center_b, 5, (255, 0, 255), -1)
                    
        #             #A[min(box_ac[:, 1]):max(box_ac[:, 1]), min(box_ac[:, 0]):max(box_ac[:, 0]), :] = I
                    
        
        # return board_corners

    def get_blue_button(self, img):
        try:
            # Find contours
            contours, _ = cv2.findContours(img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                # Select the maximum contour
                max_box = max(contours, key=cv2.contourArea)
                blue_button = cv2.minAreaRect(max_box)
                    
                return blue_button
            else:
                print("No button found.")
                return (None, None, None)
            
        except cv2.error as e:
            print(e)
            return (None, None, None)
        
        
    def get_board_position(self, board):
        # Use extrinsic parameters to estimate the 3D pose
        position = self.from2DTo3D(board[0][0], board[0][1], self.distance_camera_board_)
        
        return position
    
    def from2DTo3D(self, u, v, z):
        position = Vector3()
        position.x = (z / self.fx_) * (float(u) - self.cx_ + self.configs_.py_min)
        position.y = (z / self.fy_) * (float(v) - self.cy_ + self.configs_.px_min)
        position.z = z
        return position
    
    def get_board(self, img):
        try:
            # Find contours
            contours, _ = cv2.findContours(img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # print("Contours found:" + str(len(contours)))

            if len(contours) == 0:
                print("Contours not found. Return (None, None)")
                return (None, None)
            # Select the maximum countour
            max_box = max(contours, key=cv2.contourArea)
            board_min_area_rect = cv2.minAreaRect(max_box)
            board_corners = cv2.boxPoints(board_min_area_rect)
            print("Board")
            print(board_min_area_rect)

            # Check expected values

            # Project the board_candidate points to the starting image
            board_corners[:, 0] = board_corners[:, 0] + self.configs_.py_min
            board_corners[0, :] = board_corners[0, :] + self.configs_.px_min
            board_corners = np.int64(board_corners)
            
            return (board_min_area_rect, board_corners)
        except cv2.error as e:
            print(e)
            return (None, None)

    def preprocess_image(self, img):
        # print("Preprocess image")
        original_img = img.copy()

        # Cut image in the ROI
        img_roi = original_img[self.configs_.px_min:self.configs_.px_max-1,
                                self.configs_.py_min:self.configs_.py_max-1,:]

        # Apply hsv mask to extract the board of the pixel
        board_hsv_lower = np.array([self.configs_.board_mask_h_min,self.configs_.board_mask_s_min, self.configs_.board_mask_v_min])
        board_hsv_upper = np.array([self.configs_.board_mask_h_max,self.configs_.board_mask_s_max, self.configs_.board_mask_v_max])

        img_board_mask_hsv = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)
        img_board_mask_hsv = cv2.inRange(img_board_mask_hsv, board_hsv_lower, board_hsv_upper)
        
        # Erode and dilate
        img_board_mask_hsv = cv2.erode(img_board_mask_hsv, None, iterations=self.configs_.erode_iterations)
        img_board_mask_hsv = cv2.dilate(img_board_mask_hsv, None, iterations=self.configs_.dilate_iterations)

        return img_board_mask_hsv
        
    def get_blue_button_3D(self, original_img):
        img = original_img.copy()
        ####
        # Cut image in the ROI
        img_roi = original_img[self.configs_.px_min:self.configs_.px_max-1,
                                self.configs_.py_min:self.configs_.py_max-1,:]

        # self.show_image(4, "ROI img", img_roi)
        # Apply blue mask to detect the blue button
        blue_button_hsv_lower = np.array([self.configs_.board_blue_mask_h_min, self.configs_.board_blue_mask_s_min, self.configs_.board_blue_mask_v_min])
        blue_button_hsv_upper = np.array([self.configs_.board_blue_mask_h_max, self.configs_.board_blue_mask_s_max, self.configs_.board_blue_mask_v_max])
        
        blue_mask_img = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)
        blue_mask_img = cv2.inRange(blue_mask_img, blue_button_hsv_lower, blue_button_hsv_upper)

        # Erode and dilate
        blue_mask_img = cv2.erode(blue_mask_img, None, iterations=self.configs_.erode_iterations)
        blue_mask_img = cv2.dilate(blue_mask_img, None, iterations=self.configs_.dilate_iterations)

        self.show_image(5, "Blue mask img", blue_mask_img)

        blue_button = self.get_second_blue_blob(blue_mask_img)
        if blue_button[0] == None:
            return None
        blue_button_orig_img = (int(blue_button[0][0]),
                                int(blue_button[0][1]))
        blue_button_center3D = self.from2DTo3D(blue_button_orig_img[0], blue_button_orig_img[1], self.distance_camera_board_)
        
        cv2.circle(img, (int(blue_button[0][0]) + self.configs_.py_min,
                                int(blue_button[0][1]) + self.configs_.px_min), 13, (0, 0, 255), 2)
        # self.show_image(6, "Blue button img", img)
            
        
        print(blue_button_center3D)
        # board_pose.rotation
        
        # yaw = board
        # yaw = board[2]
        # quat = R.from_matrix([[math.cos(yaw), -math.sin(yaw), 0],
        #                             [math.sin(yaw), math.cos(yaw), 0],
        #                             [0, 0, 1]]).as_quat()
        
        # print(quat)
        # board_pose.orientation.x = quat[0]
        # board_pose.orientation.y = quat[1]
        # board_pose.orientation.z = quat[2]
        # board_pose.orientation.w = quat[3]

        # return board_pose
        return blue_button_center3D

        # print("Center: " +str(board_min_area_rect[0][0])+" "+str(board_min_area_rect[0][1]))
        # img = cv2.circle(img, (int(board_min_area_rect[0][0])+self.configs_.py_min,int(board_min_area_rect[0][1])+self.configs_.px_min),
        #                             3, (255, 0, 255), 3)
        # img = cv2.circle(img, (int(self.cx_), int(self.cy_)),
        #                             3, (100, 100, 255), 3)
        # self.show_image(10, 'Final debug image', img)
        # print(self.board_pose_)

    def get_second_blue_blob(self, img):
        try:
            # Find contours
            contours, _ = cv2.findContours(img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 1:
                # print(contours)
                # Select the maximum contour
                max_box = max(contours, key=cv2.contourArea)
                # print("First Max:")
                # print(max_box)
                try:
                    contours.remove(max_box)
                except:
                    print("Error removing contours. Returning None")
                    print("First Max:")
                    print(max_box)
                    return (None, None, None)
                # if max_box in contours_list:
                #     print("First max found. Remove element")
                    # Search for a new max
                max_box = max(contours, key=cv2.contourArea)
                
                blue_button = cv2.minAreaRect(max_box)
                
                return blue_button
                
            else:
                print("No button or display found.")
                return (None, None, None)
            
        except cv2.error as e:
            print(e)
            return (None, None, None)
        
    def get_board_corners(self):
        return self.board_corners
    
    def get_board_pose(self):
        return self.board_pose_

    def show_image(self, id, window_name, img):
        if len(img) > 0:
            if self.configs_.debug_img_to_show == -1 or self.configs_.debug_img_to_show == id:
                cv2.imshow(window_name, img)
                cv2.waitKey(1)
                if not (window_name in self.active_windows_):
                    print("Added "+window_name+" in active windows list")
                    self.active_windows_.append(window_name)
            else:
                if window_name in self.active_windows_:
                    print("Removing "+window_name+" in active windows list")
                    self.active_windows_.remove(window_name)
                    cv2.destroyWindow(window_name)