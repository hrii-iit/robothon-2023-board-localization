#! /usr/bin/env python3
import cv2 
import numpy as np
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from quaternion import from_rotation_matrix
from hrii_robothon_msgs.srv import BoardDetection,BoardDetectionResponse,DesiredSliderDisplacement,DesiredSliderDisplacementResponse
import time

global count
global W
W =[]
global act

bridge = cv_bridge.CvBridge()
count = 0


# def distance(img, slide_p , des_p):
#     h_min = 86
#     h_max = 95
#     s_min = 103
#     s_max = 255
#     v_min = 92
#     v_max = 255
#     hsv_a = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#     lower = np.array([h_min, s_min, v_min])
#     upper = np.array([h_max, s_max, v_max])

#     mask_a = cv2.inRange(hsv_a, lower, upper)
#     contours, _ = cv2.findContours(mask_a.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     center_a = None
#     c = max(contours, key=cv2.contourArea)
#     rect = cv2.minAreaRect(c)
#     ((x,y), (width, height), rotation) = rect
#     length = max([height, width])
#     l = (25.0)/length
#     x = slide_p[0] - des_p[0]
#     la = x * l
    
#     dist = la*(32.54/22.53)*(16.0/14.0)

#     return dist

    # s = f"x {np.round(x)}, y: {np.round(y)}, width: {np.round(width)}, height: {np.round(height)}, rotation: {np.round(rotation)}"
    # box = cv2.boxPoints(rect)
    # box = np.int64(box)
    # M_h = cv2.moments(c)
    
def distance(img, slide_p , des_p):
    h_min = 86
    h_max = 95
    s_min = 103
    s_max = 255
    v_min = 92
    v_max = 255
    hsv_a = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    
    
    
    
    
    mask_a = cv2.inRange(hsv_a, lower, upper)
    contours, _ = cv2.findContours(mask_a.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center_a = None
    c = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(c)
    ((x,y), (width, height), rotation) = rect
    length = max([height, width])
    l = (25.0)/length
    x = slide_p[0] - des_p[0]
    la = x * l
    
    dist = la*(32.54/22.53)*(14.7/16.0)

    return dist






def find_screen(img):
    lower1 = np.array([0, 100, 20])
    upper1 = np.array([10, 255, 255])
    
    # upper boundary RED color range values; Hue (160 - 180)
    lower2 = np.array([160,100,20])
    upper2 = np.array([179,255,255])
    Z = img.copy()
    hsv = cv2.cvtColor(Z, cv2.COLOR_BGR2HSV)
    lower_mask = cv2.inRange(hsv, lower1, upper1)
    upper_mask = cv2.inRange(hsv, lower2, upper2)  
    mask = lower_mask + upper_mask
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center = None
    c = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(c)
    ((x,y), (width, height), rotation) = rect
    s = f"x {np.round(x)}, y: {np.round(y)}, width: {np.round(width)}, height: {np.round(height)}, rotation: {np.round(rotation)}"
    box_ac = cv2.boxPoints(rect)
    box_ac = np.int64(box_ac)
    #print(f"box_ac{box_ac}")
    box_ac[:, 0] = box_ac[:, 0] 
    M = cv2.moments(c)
    I = Z[min(box_ac[:, 1]):max(box_ac[:, 1]), min(box_ac[:, 0]):max(box_ac[:, 0]), :]

    return I


def job_done(I):
    h_min = 70
    h_max = 96
    s_min = 18
    s_max = 255
    v_min = 18
    v_max = 255

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    hsv = cv2.cvtColor(I, cv2.COLOR_BGR2HSV)
    #visualize(hsv)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=6)
    mask = cv2.dilate(mask, None, iterations=6)
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours)==1:
        J = True
    else:
        J = False
    return J



def tri(temp, img):
    methods = ['cv2.TM_CCOEFF']
    cent_a =[0 , 0]
    # template = cv2.imread('temp_1.png', cv2.IMREAD_GRAYSCALE)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img2 = img.copy()
    w, h = temp.shape[::-1]
    GG = img
    import cv2 as cv
    img = img2.copy()
    for meth in methods:
        method = eval(meth)

    try:
        res = cv2.matchTemplate(img,temp,method)
        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
        top_left = max_loc
        top_left = list(top_left)
        bottom_right = [top_left[0] + w, top_left[1] + h]
        cv.rectangle(img,top_left, bottom_right, 255, 2)
        #print(bottom_right)
        #print(top_left)

        cent_a[0] = int((top_left[0]+ bottom_right[0])*0.5)
        cent_a[1] = int((top_left[1]+ bottom_right[1])*0.5)
        
    except:
        print("could not find")
        cent_a = None
        
    return cent_a, img
    




def tri_decet(temp, img):

    w, h = temp.shape[::-1]
    img2 = img.copy()
    method = "cv.TM_CCOEFF"
    act = False
    # w_u, h_u = temp_u.shape[::-1]
    try:
        res = cv2.matchTemplate(img2, temp, method=1)
        cent = [0, 0]
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        top_left_d = max_loc
        top_left_d = list(top_left_d)
        bottom_right = [top_left_d[0] + w, top_left_d[1] + h]
        cent[0] = int((top_left_d[0]+ bottom_right[0])*0.5)
        cent[1] = int((top_left_d[0]+ bottom_right[0])*0.5)
    except:
        cent = None
        print("NO triangle")

    return cent



def image_callback(data):
    global desired_displacement
    global task_accomplished
    global act


    Q = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    C = Q[int(Q.shape[0]*0.45):int(Q.shape[0]*0.45)+int(Q.shape[0]*0.2), int(Q.shape[1]*0.44):int(Q.shape[1]*0.44)+int(Q.shape[1]*0.38), :]
    I = C
    cv2.imshow("C", C)
    cv2.waitKey(1)
#screen cut required
    task_accomplished = job_done(I)

    img_a = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)

    temp_u = cv2.imread("temp_1.png", cv2.IMREAD_GRAYSCALE)
    temp_d = cv2.imread("template_a.png", cv2.IMREAD_GRAYSCALE)
    try:
        slider_pos, H_0 = tri(temp_u, I)

        try:
            desired_pos_a, H_1 = tri(temp_d, I)
            B_1 = distance(I,slider_pos, desired_pos_a)
            D = True

            #cv2.circle(I,desired_pos_a, 2, (0,0,255), -1)
            try:
                desired_pos_b, H_2 = tri(temp_d, I)
                
                B_2 = distance(I,slider_pos, desired_pos_b)
                if abs(B_1 - B_2)< 0.1:
                    raise Exception

            except:
                
                print("only one tri angle")



                
            

        except:
            print("desired pos is not available")
    except:
        slide=False
        print("slider pos not obtained")

   
    try:
        desired_displacement = max([B_1, B_2])
        W.append(B_1)
        act = True
    except:
        desired_displacement = B_1
        W.append(B_1)
        act= True


    

    #print(f"task_acc {task_accomplished} and displacement is {desired_displacement}")



def slider_desired_displacement(req):
    global task_accomplished
    global desired_displacement
    global act
    print(f"task_acc {task_accomplished} and displacement is {desired_displacement}")
    global count
    if len(W)>2 and abs(desired_displacement) + abs(W[-1])<2:
        desired_displacement = desired_displacement + 2*(desired_displacement/abs(desired_displacement))
        print("especiall case STUCK")
        
    
    print("Returning [%s]"%(desired_displacement))
    desired_displacement = desired_displacement/1000.0
    #count = count + 1
    if count>5:
        if count==5:
            desired_displacement==-0.03

        if count==6:
            desired_displacement==+0.03

        if count>6:
            print("Task not completed")

    

        # desired_displacement= 0


    #     reliable = False
    # while not camera_process_ready:
    #     rospy.loginfo("Camera process not ready, waiting...")
    #     rospy.sleep(1)

        
    count = count + 1   
    return DesiredSliderDisplacementResponse(task_accomplished, desired_displacement)
    
    

    
   

    
    


        
   
rospy.init_node("camera_subscribe")

print("Init")
rospy.wait_for_message("/camera/color/image_raw", Image, timeout=100.0)
rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
rospy.sleep(1)
#time.sleep(1)
slider_srv = rospy.Service('/robothon/slider_desired_pose', DesiredSliderDisplacement, slider_desired_displacement)
print("Service initialized")


rospy.spin()





    
    
    


    


     

    
    



