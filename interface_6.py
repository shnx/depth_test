import tkinter as tk
from tkinter import Label, Button, Checkbutton, BooleanVar
from PIL import Image, ImageTk
import numpy as np
import cv2
import urx
import threading
import pyrealsense2 as rs

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
        pose = robot.get_pose()
        position = pose.pos.array
        orientation = pose.orient.array
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

# Function to move the robot
def move_robot(dx=0, dy=0, dz=0):
    try:
        pose = robot.get_pose()
        pose.pos.x += dx
        pose.pos.y += dy
        pose.pos.z += dz
        robot.set_pose(pose, acc=0.2, vel=0.2)
    except Exception as e:
        print("Error moving robot:", e)

# Function to show RealSense camera stream
def show_camera_stream():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            
            frame = np.asanyarray(color_frame.get_data())
            
            # Apply grid overlay if enabled
            if grid_overlay_var.get():
                for i in range(1, 3):
                    cv2.line(frame, (i * frame.shape[1] // 3, 0), (i * frame.shape[1] // 3, frame.shape[0]), (0, 255, 0), 1)
                    cv2.line(frame, (0, i * frame.shape[0] // 3), (frame.shape[1], i * frame.shape[0] // 3), (0, 255, 0), 1)
            
            cv2.imshow("RealSense Camera Stream", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

# Function to start camera thread
def start_camera_thread():
    cam_thread = threading.Thread(target=show_camera_stream)
    cam_thread.daemon = True
    cam_thread.start()

# Create UI window
root = tk.Tk()
root.title("ADY Robot Control")
root.geometry("800x600")

# Load and display background image
bg_image = Image.open("ady_img.jpg")
bg_image = bg_image.resize((800, 600))
bg_photo = ImageTk.PhotoImage(bg_image)
bg_label = Label(root, image=bg_photo)
bg_label.place(relwidth=1, relheight=1)

# Labels for robot data
tcp_label = Label(root, text="TCP Position: Fetching...", font=("Arial", 12), bg="white")
tcp_label.pack(pady=10)

joints_label = Label(root, text="Joint Positions: Fetching...", font=("Arial", 12), bg="white")
joints_label.pack(pady=10)

# Movement buttons
button_frame = tk.Frame(root)
button_frame.pack(pady=10)

Button(button_frame, text="← X-", command=lambda: move_robot(dx=-0.01), bg="lightgray").grid(row=1, column=0)
Button(button_frame, text="X+ →", command=lambda: move_robot(dx=0.01), bg="lightgray").grid(row=1, column=2)
Button(button_frame, text="↑ Y+", command=lambda: move_robot(dy=0.01), bg="lightgray").grid(row=0, column=1)
Button(button_frame, text="↓ Y-", command=lambda: move_robot(dy=-0.01), bg="lightgray").grid(row=2, column=1)
Button(button_frame, text="Z+", command=lambda: move_robot(dz=0.01), bg="lightgray").grid(row=3, column=1)
Button(button_frame, text="Z-", command=lambda: move_robot(dz=-0.01), bg="lightgray").grid(row=4, column=1)

# Checkboxes for face detection and grid overlay
face_detection_var = BooleanVar()
grid_overlay_var = BooleanVar()

Checkbutton(root, text="Enable Face Detection", variable=face_detection_var).pack()
Checkbutton(root, text="Show Grid Overlay", variable=grid_overlay_var).pack()

# Camera Button
camera_button = Button(root, text="Start Camera Stream", command=start_camera_thread, font=("Arial", 12), bg="lightblue")
camera_button.pack(pady=10)

# Refresh Button
refresh_button = Button(root, text="Refresh TCP & Joint Data", command=update_ui, font=("Arial", 12), bg="lightgreen")
refresh_button.pack(pady=10)

# Start updating UI
update_ui()

# Run application
root.mainloop()
