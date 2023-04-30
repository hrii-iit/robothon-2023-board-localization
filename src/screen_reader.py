import pyrealsense2 as rs
import numpy as np
import cv2

global XX
global YY
i = 0
j = 0



def position(cent_a, j, Z):
    global XX
    global YY
    print(j)
    
    X = np.array([[cent_a[0], cent_a[1]]])
    # y_x = float(input("Enter the width from center"))
    # y_y = float(input("Enter the length from center"))
    A = ["B", "C", "D", "E", "F", "G", "H", "I"]
    dict_A = {"B": -13.9, "C": -10.45, "D": -7, "E":-3.5, "F": 0, "G":3.5, "H":9.45, "I":10.4}
    B = ["0", "1", "3", "5", "7", "9"]
    dict_B = {"0":-13, "1":-10.5, "3":-3.5, "5":3.45, "7":10.4, "9":17.35}
    
    print(f"place the board_loc on {A[(j+1)//6]}_{B[(j+1)%6]} and press Enter")
    Y = np.array([[dict_B[B[j%6]], dict_A[A[j//6]]]])
    cv2.imshow("img", Z)
    cv2.waitKey(0)
    
    cv2.destroyAllWindows()
    if j==0:
        XX = X
        YY = Y
    else:
        XX = np.concatenate((XX, X), axis=0)
        YY =np.concatenate((YY, Y), axis=0)
    j+=1
    
    return j
    






pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

h_min = 1
h_max = 30
s_min = 144
s_max = 255
v_min = 0
v_max = 255

h = 86
h_a = 110
s = 168
s_a = 255
v = 166
v_a = 255

# lower = np.array([h_min, s_min, v_min])
# upper = np.array([h_max, s_max, v_max])

# lower_se = np.array([142, s_min, v_min])
# upper_se = np.array([179, s_max, v_max])
lower_be = np.array([h, s, v])
upper_be = np.array([h_a, s_a, v_a])

first=True
second=True






while True:
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    
    Z = np.asanyarray(color_frame.get_data())
    #print(Z)

    A = Z.copy()

    hsv = cv2.cvtColor(A, cv2.COLOR_BGR2HSV)
    # mask = cv2.inRange(hsv, lower, upper)
    # mask_a = cv2.inRange(hsv, lower_se, upper_se)
    mask_b = cv2.inRange(hsv, lower_be, upper_be)

    mask =  mask_b 
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)
    try:
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center = None
        c = max(contours, key=cv2.contourArea)
        
        #c = max(range(len(contours)), key=lambda i: cv2.contourArea(contours[i]))
        
        #print(len(contours))
   
        rect = cv2.minAreaRect(c)
        ((x,y), (width, height), rotation) = rect
        s = f"x {np.round(x)}, y: {np.round(y)}, width: {np.round(width)}, height: {np.round(height)}, rotation: {np.round(rotation)}"
        box = cv2.boxPoints(rect)
        box = np.int64(box)
        M = cv2.moments(c)
        center_a = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        #print(f"{center_a} and athe shape is {A.shape}")
        cv2.circle(Z, center_a, 5, (255, 0, 255), -1)
        
        #print(f"box is {box}")
        # cv2.imshow('original', A)
        # cv2.imshow('screen', A[min(box[:, 1]):max(box[:, 1]), min(box[:, 0]):max(box[:, 0]), :])
        first = True
        if (i%60)==0 and i>5 and first==True and second==True:
            j = position(center_a, j, Z)
        #print("we reached here")
        A[min(box[:, 1]):max(box[:, 1]), min(box[:, 0]):max(box[:, 0]), :] = 0
        i+=1
    

    except:
        first = False
        cv2.imshow('original', Z)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    try:
        hsv = cv2.cvtColor(A, cv2.COLOR_BGR2HSV)
        mask_b = cv2.inRange(hsv, lower_be, upper_be)

        mask =  mask_b 
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center_b = None
        c = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        ((x,y), (width, height), rotation) = rect
        s = f"x {np.round(x)}, y: {np.round(y)}, width: {np.round(width)}, height: {np.round(height)}, rotation: {np.round(rotation)}"
        M = cv2.moments(c)
        center_b = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        cv2.circle(Z, center_b, 5, (255, 0, 255), -1)
        
        cv2.imshow('original', Z)
        second=True
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    except:
        second = False
        cv2.imshow('original', Z)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break


    if j==47:
        np.save("XX.npy", XX)
        np.save("YY.npy", YY)
        Da = np.hstack([XX, YY])
        np.save("Data.npy", Da)
        break

# print("an ast")


pipeline.stop()
cv2.destroyAllWindows()