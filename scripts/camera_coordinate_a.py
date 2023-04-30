#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from quaternion import from_rotation_matrix
import cv2
import cv_bridge
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped, Quaternion
from hrii_robothon_msgs.srv import BoardDetection,BoardDetectionResponse,DesiredSliderDisplacement,DesiredSliderDisplacementResponse
import time

import numpy as np

# TransformStamped.header.frame_id = "World"
transform_msg = TransformStamped()
publisher = rospy.Publisher('/transform_topic', TransformStamped, queue_size=10)
# Load the RGBD image
# color_image = cv2.imread("color.png")
# depth_image = cv2.imread("depth.png", cv2.IMREAD_UNCHANGED)

t = time.time()
def quaternion_from_euler(Ang):
    roll, pitch, yaw = Ang
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qx, qy, qz, qw]




def euler_to_rotation_matrix(euler_angles):
    roll, pitch, yaw = euler_angles
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    
    R_roll = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    R_pitch = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    R_yaw = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    
    R = np.dot(R_yaw, np.dot(R_pitch, R_roll))
    return R


def callback_image(data):
    global first
    global second
    global center_b
    global center_a
    global box_ac
    global t 
    h_min = 70
    h_max = 120
    s_min = 200
    s_max = 255
    v_min = 110
    v_max = 255

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    


    h = 43
    h_a = 179
    s = 70
    s_a = 255
    v = 20
    v_a = 255
 
    lower_be = np.array([h, s, v])
    upper_be = np.array([h_a, s_a, v_a])

    bridge = cv_bridge.CvBridge()
    color_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    
    #color_image = color_image[:, 100:1028, :]
    
    Z = color_image.copy()
    
    A = Z[:, 100:1028, :]
  
    hsv = cv2.cvtColor(A, cv2.COLOR_BGR2HSV)
    mask_b = cv2.inRange(hsv, lower_be, upper_be)
    mask = mask_b
    # cv2.imshow("Z",Z)
    # cv2.waitKey(1)
    
    mask = cv2.erode(mask, None, iterations=4)
    mask = cv2.dilate(mask, None, iterations=4)
    try:
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center = None
        c = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        ((x,y), (width, height), rotation) = rect
        s = f"x {np.round(x)}, y: {np.round(y)}, width: {np.round(width)}, height: {np.round(height)}, rotation: {np.round(rotation)}"
        box_ac = cv2.boxPoints(rect)
        box_ac = np.int64(box_ac)
        #print(f"box_ac{box_ac}")
        box_ac[:, 0] = box_ac[:, 0] +100
        M = cv2.moments(c)
        #center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        cv2.drawContours(Z,[box_ac],0,(0,0,255),2)
        
        # cv2.imshow('original', A)
        # cv2.waitKey(1)
        
        I = Z[min(box_ac[:, 1]):max(box_ac[:, 1]), min(box_ac[:, 0]):max(box_ac[:, 0]), :]
        #hsv_a = hsv[min(box_ac[:, 1]):max(box_ac[:, 1]), min(box_ac[:, 0]):max(box_ac[:, 0]), :]
        hsv_a = cv2.cvtColor(I, cv2.COLOR_BGR2HSV)
        cv2.imshow("hsv", hsv_a)
        mask_a = cv2.inRange(hsv_a, lower, upper)
        try:
            contours, _ = cv2.findContours(mask_a.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            center_a = None
            c = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)
            ((x,y), (width, height), rotation) = rect
            s = f"x {np.round(x)}, y: {np.round(y)}, width: {np.round(width)}, height: {np.round(height)}, rotation: {np.round(rotation)}"
            box = cv2.boxPoints(rect)
            box = np.int64(box)
            M_h = cv2.moments(c)
            
            center_a = [int(M_h["m10"] / M_h["m00"]), int(M_h["m01"] / M_h["m00"])]
            center_a[0] = center_a[0] + min(box_ac[:, 0]) 
            center_a[1] = center_a[1] + min(box_ac[:, 1])
            cv2.circle(Z, center_a, 5, (255, 0, 255), -1)
            #print(f"center_a is {center_a}")
            first = True

            try:
                hsv_a[min(box[:, 1]):max(box[:, 1]), min(box[:, 0]):max(box[:, 0]), :] = 0
                mask_c = cv2.inRange(hsv_a, lower, upper)
                contours_a, _ = cv2.findContours(mask_c.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                center_b = None
                c_a = max(contours_a, key=cv2.contourArea)
                rect_a = cv2.minAreaRect(c_a)
                ((x,y), (width, height), rotation) = rect_a
                s = f"x {np.round(x)}, y: {np.round(y)}, width: {np.round(width)}, height: {np.round(height)}, rotation: {np.round(rotation)}"
                M_a = cv2.moments(c_a)
                center_b = [int(M_a["m10"] / M_a["m00"]), int(M_a["m01"] / M_a["m00"])]
                center_b[0] = center_b[0] + min(box_ac[:, 0]) 
                center_b[1] = center_b[1] + min(box_ac[:, 1])
                cv2.circle(Z, center_b, 5, (255, 0, 255), -1)
                
                # print(f"center_b is {center_b}")
                second = True
                
                #A[min(box_ac[:, 1]):max(box_ac[:, 1]), min(box_ac[:, 0]):max(box_ac[:, 0]), :] = I
                
                cv2.imshow('original', Z)
                
                #cv2.imshow("D", D)
                print("UP TO HERE")
                cv2.imshow('screen', I)
                cv2.waitKey(1)


            except:
                cv2.imshow('screen', Z)
                print("EXXXXXXXXXXXXXXXX>E")
                cv2.waitKey(1)
                second=False
            
        except:
            cv2.imshow('screen', Z)
            cv2.waitKey(1)
            first=False


        cv2.imshow('original', Z)
        cv2.waitKey(1)

    except:
        first = False
        cv2.imshow('screen', Z)
        cv2.waitKey(1)
    
        
    # try:
    # print(f"u is {u} and v is {v}")
    #     hsv = cv2.cvtColor(A, cv2.COLOR_BGR2HSV)
    #     mask_b = cv2.inRange(hsv, lower_be, upper_be)
    #     mask =  mask_b 
    #     # mask = cv2.erode(mask, None, iterations=2)
    #     # mask = cv2.dilate(mask, None, iterations=2)
    #     contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #     center_b = None
    #     c = max(contours, key=cv2.contourArea)
    #     rect = cv2.minAreaRect(c)
    #     ((x,y), (width, height), rotation) = rect
    #     s = f"x {np.round(x)}, y: {np.round(y)}, width: {np.round(width)}, height: {np.round(height)}, rotation: {np.round(rotation)}"
    #     M = cv2.moments(c)
    #     center_b = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    #     second = True
    #     if first==True and second==True:
    #         cv2.circle(Z, center_a, 5, (255, 0, 255), -1)
    #         cv2.circle(Z, center_b, 5, (255, 0, 255), -1)
    #         cv2.imshow('Orig', Z)
    #         cv2.waitKey(1)



    # except:
    #     second = False






        















def board_location(req):
    global D
    global center_a
    global center_b
    global box_ac
    global second
    # u = np.array([center_a[0],  center_b[0]])
    # v = np.array([center_a[1], center_b[1]])
    u_1 = center_a[0]
    v_1 = center_a[1]
    u_2 = center_b[0]
    v_2 = center_b[1]
    u = np.array([[center_a[0]],  [center_b[0]]])
    v = np.array([[center_a[1]], [center_b[1]]])
    #print(f"u is {u} and v is {v}")
    # bridge = cv_bridge.CvBridge()
    # depth_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    # depth_array = np.array(depth_image, dtype=np.float32)
    # #depth_array = depth_array[:, 100:1028]
    # D = depth_array[min(box_ac[:, 1]):max(box_ac[:, 1]), min(box_ac[:, 0]):max(box_ac[:, 0])]
    # cv2.imshow("D", D)
    # cv2.waitKey(1)

    # try:
    #     print(f"center_b is {center_b}")
    # except:
    #     print("No center were found")
    # K = np.array([[904.2376098632812, 0, 647.1770629882812],
    #               [0, fy, cy],
    #               [0, 0, 1]])
    # DD = np.average(depth_array)
    cx = 628.231426
    cy = 355.775987
    fx = 941.964974
    fy = 944.968581
    # z_1 = depth_array[v_1, u_1]
    z_1 = 480
    # z_2 = depth_array[v_2, u_2]
    z_2 = 490
    x_1 = -((u_1 - cx) * z_1 / fx)+4.0
    # -(52.11-47.59)
    y_1 = ((v_1 - cy) * z_1 / fy)-7.0
    x_2 = -((u_2 - cx) * z_2 / fx)+4.0
    # -(52.11-47.59)
    y_2 = ((v_2 - cy) * z_2 / fy)-7.0

    #print(f"z of both points are {z_1} and {z_2} and difference is {abs(z_2 -z_1)}")
    print(f"distance is {x_1 , y_1} and smaller is {x_2, y_2} distance is {np.sqrt(abs(x_1 -x_2)**2 + abs(y_1 - y_2)**2)}")
    point1 = np.array([x_1, y_1])
    point2 = np.array([x_2, y_2])
    
    
    angle = (np.arctan2(point2[1] - point1[1], point2[0] - point1[0]) +np.pi)
    # + (np.pi/2) + np.deg2rad(189.4)
    
    print(f"angle is {np.rad2deg(angle)}")
    angle = angle 
    # + 0.01605703 
    euler_angles = [0, 0, angle]
    O = np.array([0.0, 0.0, 0.0, 1.00]).reshape((4,1))
    translation = np.array([[x_1*0.001, y_1*0.001, 100.0*0.001, 1.00]])

    R = euler_to_rotation_matrix(euler_angles)
    [qx, qy, qz, qw] = quaternion_from_euler(euler_angles)

    

    R_a = np.vstack((R, np.array([[0, 0, 0]])))
    translation_o = np.array([[-0.024+0.00815, -0.097, -100.0*0.001, 1.00]])
    T_M = np.hstack((R_a , translation.T))
    R_O = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    R_O = np.vstack((R_O, np.array([[0, 0, 0]])))
    T_O = np.hstack((R_O , translation_o.T))
    
    T_T = np.dot(T_M, T_O)
    O_m = np.dot(T_M, O)
    O_new = np.dot(T_T, O)
    R_A = T_T[0:3, 0:3]
    q = from_rotation_matrix(R_A)
    # q = q.as_quaternion(q)
    transform_msg.header.frame_id = 'world'
    transform_msg.child_frame_id = 'task_board_base_link'
    transform_msg.header.stamp = rospy.Time.now()
    # print(f"shape is {T_T.shape}")

    transform_msg.transform.translation.x = O_new[0, 0]
    transform_msg.transform.translation.y = O_new[1, 0]
    transform_msg.transform.translation.z = O_new[2, 0]
    transform_msg.transform.rotation.x = q.x
    transform_msg.transform.rotation.y = q.y
    transform_msg.transform.rotation.z = q.z
    transform_msg.transform.rotation.w = q.w
     
    if second==True:
        print(f"world to base is ")
        print(T_T)
        print(f"position of base {O_new}")

        print("world to screen")
        print(T_M)
        print(f"screen pose {O_m}")

        br = tf2_ros.StaticTransformBroadcaster()
        br.sendTransform(transform_msg)

        success = True
        return BoardDetectionResponse(success)



    

    


    


    print(f"O_new is {O_new}")
    # print(T_M)
    # axis1 = rotation_matrix.dot(np.array([1, 0]))
    # axis2 = rotation_matrix.dot(np.array([0, 1]))
    
    # depth_meters = depth_array * 0.001 # Convert to meters
    
    # a = depth_array[0]
    # b = depth_array[1]

    #print(depth_meters[220:222, 400:402])
# Obtain the camera intrinsic matrix




# K = np.array([[fx, 0, cx],
#               [0, fy, cy],
#               [0, 0, 1]])

# # Convert the 2D image coordinates to 3D camera coordinates
# u = 320
# v = 240
# z = depth_image[v, u] / 1000.0 # Convert depth from millimeters to meters
# x = (u - cx) * z / fx
# y = (v - cy) * z / fy
# point_camera = np.array([x, y, z, 1]).T

# # Convert the 3D camera coordinates to 3D world coordinates
# T = np.loadtxt("transform.txt") # Load the transformation matrix
# point_world = T.dot(point_camera)


rospy.init_node("camera_subscribe")

rospy.Subscriber("/d435_camera/color/image_raw", Image, callback_image)
#board_detection_srv = rospy.Service('/fsm/board_detection', BoardDetection, board_location)
#rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, board_location)

rospy.spin()

