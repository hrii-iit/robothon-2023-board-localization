import tkinter as tk
import numpy as np

class VoltmeterAnimation:
    
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(master, width=200, height=200)
        self.canvas.pack()
        self.draw_voltmeter()
        self.draw_indicator()
        self.start_animation()
        
    def draw_voltmeter(self):
        # Draw voltmeter background
        self.canvas.create_arc(10, 10, 190, 190, start=30, extent=120, style="arc", width=10, outline="#888")
        # Draw voltmeter ticks
        for angle in range(30, 151, 10):
            x1 = 100 + 80 * np.cos(angle * np.pi / 180)
            y1 = 100 + 80 * np.sin(angle * np.pi / 180)
            x2 = 100 + 90 * np.cos(angle * np.pi / 180)
            y2 = 100 + 90 * np.sin(angle * np.pi / 180)
            self.canvas.create_line(x1, y1, x2, y2, width=2, fill="#888")
            
    def draw_indicator(self):
        # Draw indicator line
        self.indicator = self.canvas.create_line(100, 100, 100, 20, width=3, fill="red")
        
    def start_animation(self):
        self.value = 0
        self.update_indicator()
        
    def update_indicator(self):
        # Calculate the angle of the indicator based on the value
        angle = 30 + 1.2 * self.value
        x = 100 + 70 * np.cos(angle * np.pi / 180)
        y = 100 + 70 * np.sin(angle * np.pi / 180)
        # Update the position of the indicator
        self.canvas.coords(self.indicator, 100, 100, x, y)
        # Increment the value and schedule the next update
        self.value += 1
        if self.value <= 100:
            self.master.after(50, self.update_indicator)

root = tk.Tk()
animation = VoltmeterAnimation(root)
root.mainloop()
