import cv2
import tkinter as tk
from PIL import ImageTk, Image

def exit_app():
    window.destroy()
# Load the image
image = cv2.imread("/home/hami/screen_remote.png")
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Create a tkinter window
window = tk.Tk()

# Set the window title
window.title("Image and Text Visualization")

# Set the window size
window.geometry("800x400")

# Convert the image to PIL format
img = Image.fromarray(image)
img = img.resize((int(image.shape[1]*0.5), int(image.shape[0]*0.5)), Image.ANTIALIAS)  # resize the image to fit the window
img = ImageTk.PhotoImage(img)

# Create a label for the image
img_label = tk.Label(window, image=img)
img_label.pack(side="right")

# Create a label for the text
text_label = tk.Label(window, text="Enter your text here", font=("Arial", 16))
text_label.pack(side="left")

# Start the main event loop
exit_button = tk.Button(window, text="Exit", command=exit_app)
exit_button.pack(side="bottom")

window.mainloop()