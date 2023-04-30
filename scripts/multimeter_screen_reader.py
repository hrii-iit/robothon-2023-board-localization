#! /usr/bin/env python3
import io
import os
from google.cloud import vision_v1 as vision
from google.cloud.vision_v1 import types
import matplotlib.pyplot as plt
import cv2
import numpy as np
import rospy 
import cv_bridge
import time
import sensor_msgs
import cv2
import tkinter as tk
from PIL import ImageTk, Image
from hrii_robothon_msgs.srv import ScreenVoltage

# bridge = cv_bridge.CvBridge()
global v
global first
global second
global A

second = True

first = False



def vis(img_a, voltage):
    global window
    window = tk.Tk()
    window.title("voltage_and_test_result")
    window.geometry("1280x400")
    image = cv2.cvtColor(img_a, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(image)
    img = img.resize((int(img_a.shape[1]*0.5), int(img_a.shape[0]*0.5)), Image.ANTIALIAS)  # resize the image to fit the window
    img = ImageTk.PhotoImage(img)

    img_label = tk.Label(window, image=img)
    img_label.pack(side="right")
    if abs(voltage)>2.9:
        battery_status=True
        text_label = tk.Label(window, text=f"The battery's charge level meets the required specifications for operation. and the voltage is {voltage}", font=("Arial", 16), fg="green")
    else:
        battery_status=False
        text_label = tk.Label(window, text=f"battery replacement required, voltage is {voltage}", font=("Arial", 16), fg="red")

    text_label.pack(side="left")
    exit_button = tk.Button(window, text="Exit", command=exit_app)
    exit_button.pack(side="bottom")
    window.mainloop()





def exit_app():
    global window
    window.destroy()

def image_callback(data):
    global desired_displacement
    global second
    global task_accomplished
    global v
    global act
    global A
    global first
    global Q
    bridge = cv_bridge.CvBridge()
    
    if first and second: 
        time.sleep(2)
        try:

            print(f"voltage is {v}")
            vis(A, v)
            second = False
        except:
            print("V not ready")
        

    Q = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    cv2.imshow("Q", Q)
    cv2.waitKey(1)


    ### detect the screen and cut it out 
    ##preprocessing on Image



def read_text(req):
    global v
    global Q
    global A
    A = Q.copy()
    global first
    global second
    
    first= True
    
    
    
    os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/iit.local/mleonori/Downloads/positive-cacao-379821-a0b54911257f.json'
    client = vision.ImageAnnotatorClient()
    retval, buffer = cv2.imencode('.jpg', A)
    content = buffer.tobytes()
    image = types.Image(content=content)
    response = client.text_detection(image=image)
    texts = response.text_annotations
    print(len(texts))
    a = response.full_text_annotation.text
    print((a[-8:-3]))
    for text in texts[1:]:
        vertices = [(vertex.x, vertex.y) for vertex in text.bounding_poly.vertices]
        cv2.polylines(A, [np.array(vertices)], True, (0, 255, 0), thickness=2)


    
    Z = []
    for i in range(len(texts)):
        try:
            z = float(texts[i].description)
            Z.append(z)
            #print(i)
        except:
            print("not a number")
 
    for i in range(len(Z)):
        if Z[i]<3.5:
            if Z[i]>-3.5:
                v=Z[i]
                break
    # if first and second: 
    #     print("")
    #     print(f"voltage is {v}")
    #     vis(Q, v)
    #     second = False


# Display the image with bounding boxes
    #time.sleep(2)
    return v
    # cv2.imshow("OCR Result", A)
    # cv2.waitKey(100)
    # cv2.destroyAllWindows()





# Instantiate a client
# os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/hami/Downloads/positive-cacao-379821-a0b54911257f.json'
# #client = vision.ImageAnnotatorClient()

# # Load the image file into memory
# client = vision.ImageAnnotatorClient()

# # Load the image file into memory
# with io.open("/home/hami/Downloads/photo1682515976.jpeg", "rb") as image_file:
#     content = image_file.read()
# image = types.Image(content=content)

# Perform OCR on the image file
# response = client.text_detection(image=image)
# texts = response.text_annotations
# print(len(texts))
# a = response.full_text_annotation.text

# print((a[-8:-3]))
# img = cv2.imread("/home/hami/Downloads/photo1682515976.jpeg")

# # Draw bounding boxes around the detected text
# for text in texts[1:]:
#     vertices = [(vertex.x, vertex.y) for vertex in text.bounding_poly.vertices]
#     cv2.polylines(img, [np.array(vertices)], True, (0, 255, 0), thickness=2)

# # Display the image with bounding boxes
# cv2.imshow("OCR Result", img)
# cv2.destroyAllWindows()

# AIzaSyCw6AFe5-Raz8N3sSf1Ecv6cIlgDxDLD6w




rospy.init_node("screen_reader")
rospy.wait_for_message("/camera/color/image_raw", sensor_msgs.msg.Image, timeout=100.0)

rospy.Subscriber("/camera/color/image_raw", sensor_msgs.msg.Image, image_callback)
rospy.sleep(1)

read_serv = rospy.Service('/robothon/read_voltage', ScreenVoltage, read_text)
print("Service initialized")
rospy.spin()
