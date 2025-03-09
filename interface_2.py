import tkinter as tk
from tkinter import Label, Button
from PIL import Image, ImageTk
import numpy as np
import cv2
import urx
import threading

# Default Robot IP
ROBOT_IP = "192.168.2.104"

# Connect to the robot
def connect_robot():
    try:
        return urx.Robot(ROBOT_IP)
    except Exception as e:
        print("Error connecting to robot:", e)
        return None

robot = connect_robot()

# Function to get joint positions in degrees
def get_joint_positions():
    try:
        joints = robot.getj()
        return [np.degrees(j) for j in joints]
    except Exception as e:
        print("Error getting joint positions:", e)
        return None

# Function to get TCP pose
def get_tcp_pose():
    try:
        pose = robot.get_pose()  # Get Transform object
        position = pose.pos.array  # Get position (x, y, z)
        orientation = pose.orient.array  # Get rotation matrix
        return position, orientation
    except Exception as e:
        print("Error getting TCP pose:", e)
        return None, None

# Function to update UI labels
def update_ui():
    position, orientation = get_tcp_pose()
    joints = get_joint_positions()
    
    if position is not None and orientation is not None:
        tcp_label.config(text=f"TCP Position:\n{position}\n\nRotation Matrix:\n{orientation}")
    if joints is not None:
        joints_label.config(text=f"Joints (Degrees):\n{np.round(joints, 2)}")
    
    root.after(1000, update_ui)

# Function to show camera stream with depth overlay
def show_camera_stream():
    cap = cv2.VideoCapture(0)  # Change index if needed
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Simulate depth overlay (replace with real Intel RealSense depth data if available)
        overlay = np.zeros_like(frame)
        overlay[:, :, 2] = 150  # Red tint as an example
        frame = cv2.addWeighted(frame, 0.7, overlay, 0.3, 0)
        
        cv2.imshow("Camera Stream with Depth", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

# Function to start camera thread
def start_camera_thread():
    cam_thread = threading.Thread(target=show_camera_stream)
    cam_thread.daemon = True
    cam_thread.start()

# Create UI window
root = tk.Tk()
root.title("ADY Robot Interface")
root.geometry("800x600")

# Load and display background image
bg_image = Image.open("ady_img.jpg")
bg_image = bg_image.resize((800, 600))
bg_photo = ImageTk.PhotoImage(bg_image)
bg_label = Label(root, image=bg_photo)
bg_label.place(relwidth=1, relheight=1)

# Labels for robot data
tcp_label = Label(root, text="TCP Position: ", font=("Arial", 12), bg="white")
tcp_label.pack(pady=10)

joints_label = Label(root, text="Joints: ", font=("Arial", 12), bg="white")
joints_label.pack(pady=10)

# Buttons
camera_button = Button(root, text="Show Camera Stream", command=start_camera_thread, font=("Arial", 12), bg="lightblue")
camera_button.pack(pady=10)

# Start updating UI
tk.Button(root, text="Refresh Data", command=update_ui, font=("Arial", 12), bg="lightgreen").pack(pady=10)
update_ui()

# Run application
root.mainloop()
