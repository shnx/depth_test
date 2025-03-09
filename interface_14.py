import tkinter as tk
from tkinter import Label, Button, Checkbutton, BooleanVar, Frame
from PIL import Image, ImageTk
import numpy as np
import cv2
import urx
import threading
import pyrealsense2 as rs
import mediapipe as mp

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
        tcp_label.config(text=f"TCP Position (mm):\n{np.round(position, 3)}\n\nRotation Matrix:\n{np.round(orientation, 3)}")
    if joints is not None:
        joints_label.config(text=f"Joint Positions (deg):\n{np.round(joints, 2)}")
    
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

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Initialize MediaPipe Face Detection
mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.5)

# Global variables for streaming control
streaming = True

# Function to show RealSense camera stream
def show_camera_stream():
    global streaming
    try:
        while streaming:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Apply face detection if enabled
            if face_detection_var.get():
                rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                results = face_detection.process(rgb_image)
                if results.detections:
                    for detection in results.detections:
                        bboxC = detection.location_data.relative_bounding_box
                        ih, iw, _ = color_image.shape
                        x, y, w, h = int(bboxC.xmin * iw), int(bboxC.ymin * ih), int(bboxC.width * iw), int(bboxC.height * ih)
                        cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Apply grid overlay if enabled
            if grid_overlay_var.get():
                rows, cols = 4, 4  # Increase grid points to 16 (4x4 grid)
                for i in range(1, cols):
                    for j in range(1, rows):
                        x = i * color_image.shape[1] // cols
                        y = j * color_image.shape[0] // rows
                        
                        # Draw grid lines
                        cv2.line(color_image, (x, 0), (x, color_image.shape[0]), (0, 255, 0), 1)
                        cv2.line(color_image, (0, y), (color_image.shape[1], y), (0, 255, 0), 1)
                        
                        # Display depth at grid points
                        depth_value = depth_frame.get_distance(x, y)
                        if depth_value == 0:
                            depth_value_text = "N/A"
                            depth_color = (255, 165, 0)  # Orange color
                        else:
                            depth_value_text = f"{depth_value:.2f}m"
                            depth_color = (255, 165, 0)  # Orange color
                        
                        cv2.putText(color_image, depth_value_text, (x + 5, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, depth_color, 2)

            # Display the frame
            cv2.imshow("RealSense Camera Stream", color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

# Function to start camera thread
def start_camera_thread():
    global streaming
    streaming = True
    cam_thread = threading.Thread(target=show_camera_stream)
    cam_thread.daemon = True
    cam_thread.start()

# Function to stop camera stream
def stop_camera_stream():
    global streaming
    streaming = False
    pipeline.stop()
    cv2.destroyAllWindows()

# Create UI window
root = tk.Tk()
root.title("ADY Robot Control")
root.geometry("1000x700")
root.configure(bg='#2E3440')  # Dark background

# Custom Fonts
title_font = ("Arial", 18, "bold")
label_font = ("Arial", 12)
button_font = ("Arial", 10, "bold")

# Background Image
bg_image = Image.open("ady_img.jpg").resize((1000, 700))
bg_photo = ImageTk.PhotoImage(bg_image)
bg_label = Label(root, image=bg_photo)
bg_label.place(relwidth=1, relheight=1)

# Frame for Controls
control_frame = Frame(root, bg='#3B4252', bd=2, relief=tk.RIDGE)
control_frame.place(x=700, y=50, width=280, height=600)

# Title Label
title_label = Label(control_frame, text="Robot Control Panel", font=title_font, fg='white', bg='#3B4252')
title_label.pack(pady=10)

# TCP Position Label
tcp_label = Label(control_frame, text="TCP Position (mm):\nFetching...", font=label_font, fg='white', bg='#3B4252')
tcp_label.pack(pady=10)

# Joint Positions Label
joints_label = Label(control_frame, text="Joint Positions (deg):\nFetching...", font=label_font, fg='white', bg='#3B4252')
joints_label.pack(pady=10)

# Movement Buttons
button_frame = Frame(control_frame, bg='#3B4252')
button_frame.pack(pady=10)

Button(button_frame, text="← X-", command=lambda: move_robot(dx=-0.01), bg="#4C566A", fg="white").grid(row=1, column=0, padx=5, pady=5)
Button(button_frame, text="X+ →", command=lambda: move_robot(dx=0.01), bg="#4C566A", fg="white").grid(row=1, column=2, padx=5, pady=5)
Button(button_frame, text="↑ Y+", command=lambda: move_robot(dy=0.01), bg="#4C566A", fg="white").grid(row=0, column=1, padx=5, pady=5)
Button(button_frame, text="↓ Y-", command=lambda: move_robot(dy=-0.01), bg="#4C566A", fg="white").grid(row=2, column=1, padx=5, pady=5)
Button(button_frame, text="Z+", command=lambda: move_robot(dz=0.01), bg="#4C566A", fg="white").grid(row=3, column=1, padx=5, pady=5)
Button(button_frame, text="Z-", command=lambda: move_robot(dz=-0.01), bg="#4C566A", fg="white").grid(row=4, column=1, padx=5, pady=5)

# Checkboxes for face detection and grid overlay
face_detection_var = BooleanVar()
grid_overlay_var = BooleanVar()

Checkbutton(control_frame, text="Enable Face Detection", variable=face_detection_var, font=label_font, fg='white', bg='#3B4252', selectcolor='#4C566A').pack(pady=5)
Checkbutton(control_frame, text="Show Grid Overlay", variable=grid_overlay_var, font=label_font, fg='white', bg='#3B4252', selectcolor='#4C566A').pack(pady=5)

# Camera Buttons
camera_button = Button(control_frame, text="Start Camera Stream", command=start_camera_thread, font=button_font, bg="#5E81AC", fg="white")
camera_button.pack(pady=20)

stop_camera_button = Button(control_frame, text="Stop Camera Stream", command=stop_camera_stream, font=button_font, bg="#BF616A", fg="white")
stop_camera_button.pack(pady=20)

# Start updating UI
update_ui()

# Run application
root.mainloop()
