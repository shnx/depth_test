import tkinter as tk
from tkinter import Label, Button, Checkbutton, BooleanVar
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

# Function to get TCP pose (Restored to the working version)
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
    global pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    # Initialize MediaPipe for face detection
    mp_face_detection = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.2)
    stop_streaming[0] = False

    try:
        while True:
            if stop_streaming[0]:
                break
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue

            color_frame_data = np.asanyarray(color_frame.get_data())
            depth_frame_data = np.asanyarray(depth_frame.get_data())

            # Apply grid overlay and depth values
            if grid_overlay_var.get():
                for i in range(1, 4):  # 3x3 grid
                    for j in range(1, 4):
                        x = (i * color_frame_data.shape[1]) // 3
                        y = (j * color_frame_data.shape[0]) // 3
                        
                        # Clip to ensure coordinates are within image bounds
                        x = min(max(x, 0), color_frame_data.shape[1] - 1)
                        y = min(max(y, 0), color_frame_data.shape[0] - 1)

                        # Draw grid lines
                        cv2.line(color_frame_data, (x, 0), (x, color_frame_data.shape[0]), (0, 255, 0), 1)
                        cv2.line(color_frame_data, (0, y), (color_frame_data.shape[1], y), (0, 255, 0), 1)

                        # Fetch depth value at grid cell
                        depth_value = depth_frame_data[y, x]
                        cv2.putText(color_frame_data, f"{depth_value}", (x + 5, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Face Detection
            if face_detection_var.get():
                frame_rgb = cv2.cvtColor(color_frame_data, cv2.COLOR_BGR2RGB)
                results = mp_face_detection.process(frame_rgb)
                
                if results.detections:
                    for detection in results.detections:
                        bboxC = detection.location_data.relative_bounding_box
                        ih, iw, _ = color_frame_data.shape
                        x, y, w, h = int(bboxC.xmin * iw), int(bboxC.ymin * ih), int(bboxC.width * iw), int(bboxC.height * ih)
                        cv2.rectangle(color_frame_data, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cv2.imshow("RealSense Camera Stream", color_frame_data)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_streaming[0] = True
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

# Function to stop camera stream
def stop_camera_stream():
    stop_streaming[0] = True
    cv2.destroyAllWindows()

# Function to start camera thread
def start_camera_thread():
    stop_streaming[0] = False
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

# Camera Buttons
camera_button = Button(root, text="Start Camera Stream", command=start_camera_thread, font=("Arial", 12), bg="lightblue")
camera_button.pack(pady=10)

stop_button = Button(root, text="Stop Camera Stream", command=stop_camera_stream, font=("Arial", 12), bg="lightcoral")
stop_button.pack(pady=10)

# Refresh Button
refresh_button = Button(root, text="Refresh TCP & Joint Data", command=update_ui, font=("Arial", 12), bg="lightgreen")
refresh_button.pack(pady=10)

# Shared stop streaming variable
stop_streaming = [False]  # Use a list to allow modification within thread

# Start updating UI
update_ui()

# Run application
root.mainloop()
