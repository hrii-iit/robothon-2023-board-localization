#! /usr/bin/env python3

import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2

from geometry_msgs.msg import Pose, Vector3, Quaternion
from hrii_board_localization.cfg import board_localization_paramConfig

class VisualServoing:
    def __init__(self):
        print("VisualServoing")
        self.configs_ = None
        self.camera_info_ = None
        self.board_pose_ = Pose()
        self.distance_camera_board_ = 0.38
        self.active_windows_ = list()

    def set_configs(self, configs):
        self.configs_ = configs

    def set_camera_info(self, camera_info):
        self.camera_info_ = camera_info
        self.fx_ = self.camera_info_.K[0]
        self.fy_ = self.camera_info_.K[4]
        self.cx_ = self.camera_info_.K[2]
        self.cy_ = self.camera_info_.K[5]

    # def localize_blue_button(self, image_raw):
        
    #     self.localize_blue_button_px(image_raw)

    # def localize_blue_button_px(self, image_raw):
        
    #     self.show_image(11, 'original image', image_raw)

    #     return self.compute_blue_button_center(image_raw)
    
    def get_feature_displacement(self, image_raw, feature):
        if feature == "task_board_blue_button":
            return self.get_blue_button_displacement(image_raw)
        elif feature == "task_board_red_button":
            return self.get_red_button_displacement(image_raw)
        else:
            print("Feature not recognized. Returning (None, None)")
            return (None, None)

    def get_red_button_displacement(self, image_raw):
        # Cut image in the ROI
        img_roi = image_raw[self.configs_.red_mask_px_min:self.configs_.red_mask_px_max-1,
                                self.configs_.red_mask_py_min:self.configs_.red_mask_py_max-1,:]
        # self.show_image(19, 'ROI image', img_roi)
        red_button_pos_px = self.compute_red_button_center(img_roi)
        # red_button_pos_px = self.compute_red_button_center(image_raw)
        if red_button_pos_px == None:
            return None
        return (int(self.cx_-red_button_pos_px[0]-self.configs_.red_mask_py_min), int(self.cy_-red_button_pos_px[1]-self.configs_.red_mask_px_min))
        # return (int(self.cx_-red_button_pos_px[0]), int(self.cy_-red_button_pos_px[1]))

    def get_blue_button_displacement(self, image_raw):
        blue_button_pos_px = self.compute_blue_button_center(image_raw)
        if blue_button_pos_px == None:
            return None
        return (int(self.cx_-blue_button_pos_px[0]), int(self.cy_-blue_button_pos_px[1]))

    def compute_blue_button_center(self, original_img):
        img = original_img.copy()

        blue_button_px = self.compute_blue_button_px(img)
        if blue_button_px == None:
            return None

        img = cv2.circle(img, (int(blue_button_px[0][0]),int(blue_button_px[0][1])), 3, (255, 0, 255), 3)
        # Draw target (image center)
        img = cv2.circle(img, (int(self.cx_), int(self.cy_)), 3, (0, 0, 255), 20)
        img = cv2.circle(img, (int(self.cx_), int(self.cy_)), 3, (255, 255, 255), 10)
        img = cv2.circle(img, (int(self.cx_), int(self.cy_)), 3, (0, 0, 255), 5)

        self.show_image(20, 'Final debug image', img)
        return blue_button_px[0]
    
    def compute_blue_button_px(self, img):
        
        # Apply blue mask to detect the blue button
        blue_button_hsv_lower = np.array([self.configs_.blue_mask_h_min, self.configs_.blue_mask_s_min, self.configs_.blue_mask_v_min])
        blue_button_hsv_upper = np.array([self.configs_.blue_mask_h_max, self.configs_.blue_mask_s_max, self.configs_.blue_mask_v_max])
        
        blue_mask_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        blue_mask_img = cv2.inRange(blue_mask_img, blue_button_hsv_lower, blue_button_hsv_upper)

        # Erode and dilate
        blue_mask_img = cv2.erode(blue_mask_img, None, iterations=self.configs_.erode_iterations)
        blue_mask_img = cv2.dilate(blue_mask_img, None, iterations=self.configs_.dilate_iterations)

        self.show_image(12, "Blue mask img", blue_mask_img)

        blue_button = self.get_button(blue_mask_img)

        return blue_button
    

    def compute_red_button_center(self, original_img):
        img = original_img.copy()

        red_button_px = self.compute_red_button_px(img)
        if red_button_px == None:
            return None

        img = cv2.circle(img, (int(red_button_px[0][0]),int(red_button_px[0][1])), 3, (255, 0, 255), 3)
        # Draw target (image center)
        img = cv2.circle(img, (int(self.cx_), int(self.cy_)), 3, (0, 0, 255), 20)
        img = cv2.circle(img, (int(self.cx_), int(self.cy_)), 3, (255, 255, 255), 10)
        img = cv2.circle(img, (int(self.cx_), int(self.cy_)), 3, (0, 0, 255), 5)

        self.show_image(20, 'Final debug image', img)
        return red_button_px[0]
    
    def compute_red_button_px(self, img):
        
        # Apply red mask to detect the red button
        red_button_hsv_lower = np.array([self.configs_.red_mask1_h_min, self.configs_.red_mask1_s_min, self.configs_.red_mask1_v_min])
        red_button_hsv_upper = np.array([self.configs_.red_mask1_h_max, self.configs_.red_mask1_s_max, self.configs_.red_mask1_v_max])
        
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        red_mask1 = cv2.inRange(img_hsv, red_button_hsv_lower, red_button_hsv_upper)

        # self.show_image(15, "Red mask 1 img", red_mask1)

        red_button_hsv_lower = np.array([self.configs_.red_mask2_h_min, self.configs_.red_mask2_s_min, self.configs_.red_mask2_v_min])
        red_button_hsv_upper = np.array([self.configs_.red_mask2_h_max, self.configs_.red_mask2_s_max, self.configs_.red_mask2_v_max])
        
        red_mask2 = cv2.inRange(img_hsv, red_button_hsv_lower, red_button_hsv_upper)

        # self.show_image(16, "Red mask2 img", red_mask2)
        ## Old approach
        # red_mask = red_mask1 + red_mask2
        # output_img = img_hsv.copy()
        # output_img[np.where(red_mask==0)] = 0

        # self.show_image(18, "Red mask img", output_img)

        # h,s,v = cv2.split(output_img)
        # output_img_gray = output_img[2]
        # # img_bgr = cv2.cvtColor(output_img, cv2.COLOR_HSV2GRAY)
        # self.show_image(19, "h bgr", h)
        # self.show_image(20, "s bgr", s)
        # self.show_image(21, "v bgr", v)

        # # self.show_image(22, "output_img_gray", output_img_gray)

        # # Erode and dilate
        # img_bgr = cv2.erode(v, None, iterations=self.configs_.red_erode_iterations)
        # img_bgr = cv2.dilate(img_bgr, None, iterations=self.configs_.red_dilate_iterations)

        # self.show_image(17, "Red mask erode/dilate img", img_bgr)

        # red_button = self.get_button(img_bgr)

        ## New approach
        red_mask = red_mask1 + red_mask2
        # self.show_image(17, "Red mask img", red_mask)

        # Erode and dilate
        img_red = cv2.erode(red_mask, None, iterations=self.configs_.red_erode_iterations)
        img_red = cv2.dilate(img_red, None, iterations=self.configs_.red_dilate_iterations)

        # self.show_image(17, "Red mask img", img_red)

        contours,_ = cv2.findContours(img_red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        areas = [cv2.contourArea(c) for c in contours]
        sorted_areas = np.sort(areas)
        cnt = contours[areas.index(sorted_areas[-1])] #the biggest contour
        

        ellipse = cv2.fitEllipse(cnt)
        img_c = img.copy()
        cv2.ellipse(img_c, ellipse, (255,0,0), 2)
        # print("ellipse")
        # print(ellipse)

        self.show_image(17, "Img and ellipse", img_c)
        # red_button = None
        return ellipse


    def get_button(self, img):
        try:
            # Find contours
            contours, _ = cv2.findContours(img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                # Select the maximum countour
                max_box = max(contours, key=cv2.contourArea)
                button = cv2.minAreaRect(max_box)
                
                return button
            else:
                print("No button found.")
                return None

        except cv2.error as e:
            print(e)
            return (None, None, None)
    
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

    def show_image(self, id, window_name, img):
        if self.configs_.debug_img_to_show == -1 or self.configs_.debug_img_to_show == id:
            cv2.imshow(window_name, img)
            cv2.waitKey(1)
            if not (window_name in self.active_windows_):
                print("Added %s in active windows list", window_name)
                self.active_windows_.append(window_name)
        else:
            if window_name in self.active_windows_:
                print("Added %s in active windows list", window_name)
                self.active_windows_.remove(window_name)
                cv2.destroyWindow(window_name)