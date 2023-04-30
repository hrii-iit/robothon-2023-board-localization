import io
import os
from google.cloud import vision_v1 as vision
from google.cloud.vision_v1 import types
import matplotlib.pyplot as plt
import cv2
import numpy as np
import rospy 
import cv_bridge
from sensor_msgs.msg import Image
import tkinter as tk

from PIL import ImageTk, Image

def exit_app():
    global window
    window.destroy()

def vis(img_a, voltage):
    global window
    window = tk.Tk()
    window.title("voltage_and_test_result")
    window.geometry("1200x620")
    image = cv2.cvtColor(img_a, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(image)
    img = img.resize((int(img_a.shape[1]*0.85), int(img_a.shape[0]*0.85)), Image.ANTIALIAS)  # resize the image to fit the window
    img = ImageTk.PhotoImage(img)

    img_label = tk.Label(window, image=img)
    img_label.pack(side="right")
    if abs(voltage)>2.9:
        battery_status=True
        text_label = tk.Label(window, text=f" \n \n The voltage is {voltage}", font=("Times New Roman", 24), fg="green")
    else:
        battery_status=False
        text_label = tk.Label(window, text=f"\n The voltage is {voltage}", font=("Times New Roman", 24), fg="red")

    text_label.pack(side="left")
    exit_button = tk.Button(window, text="Exit", command=exit_app)
    exit_button.pack(side="bottom")
    window.mainloop()


os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/hami/Downloads/positive-cacao-379821-a0b54911257f.json'
#client = vision.ImageAnnotatorClient()

# Load the image file into memory
client = vision.ImageAnnotatorClient()
img = cv2.imread("/home/hami/screen_remote.png", 1)

A = img.copy()
img_a = cv2.rotate(A, cv2.ROTATE_180)

print(A.shape)
## define area of the picture
A = img_a[0:200, 150:600, :]
cv2.imshow("A", A)
cv2.waitKey(0)
retval, buffer = cv2.imencode('.jpg', A)
content = buffer.tobytes()
image = types.Image(content=content)
# Load the image file into memory
# with io.open("/home/hami/Downloads/photo1682515976.jpeg", "rb") as image_file:
#     content = image_file.read()
# image = types.Image(content=content)

# Perform OCR on the image file
response = client.text_detection(image=image)
texts = response.text_annotations
print(len(texts))
a = response.full_text_annotation.text

print((a))
#img = cv2.imread("/home/hami/screen_remote.png")

# Draw bounding boxes around the detected text
for text in texts[1:]:
    vertices = [(vertex.x, vertex.y) for vertex in text.bounding_poly.vertices]
    cv2.polylines(A, [np.array(vertices)], True, (0, 255, 0), thickness=2)

# Display the image with bounding boxes
##redefine the same area
img_a[0:200, 150:600, :]=A
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





print(f"voltage is {v}")
cv2.imshow("OCR Result", img_a)
cv2.waitKey(0)
# v= 2.7
vis(img_a, v)
cv2.destroyAllWindows() 