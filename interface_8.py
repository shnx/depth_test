import tkinter as tk
from tkinter import ttk
import cv2
import numpy as np
from PIL import Image, ImageTk
import threading
import pyrealsense2 as rs
import mediapipe as mp
import socket
from time import sleep
import urx  # For real robot data

# Default Robot IP and Port
ROBOT_IP = "192.168.2.104"
ROBOT_PORT = 30002

# Connect to the robot via socket
robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
robot_socket.connect((ROBOT_IP, ROBOT_PORT))

# Function to send commands to the robot
def send_robot_command(command):
    robot_socket.send(command.encode())
    print(f"Sent command: {command}")

# Set the robot TCP and payload
send_robot_command("set_tcp([0, 0, 0.1, 0, 0, 0])\n")
send_robot_command("set_payload(0.1, [0, 0, 0.1])\n")
sleep(0.2)  # Allow some time for the robot to process the setup commands

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Initialize MediaPipe Face Detection
mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.2)

# Global variables
streaming = True
show_depth = False  # Toggle for depth overlay
show_face = False  # Toggle for face detection
show_grid = False  # Toggle for 3x3 depth grid

# Tkinter Window
root = tk.Tk()
root.title("ADY Robot Control")
root.geometry("1200x800")
root.configure(bg='#2E3440')  # Dark background

# Custom Fonts
title_font = ("Arial", 18, "bold")
label_font = ("Arial", 12)
button_font = ("Arial", 10, "bold")

# Background Image
bg_image = Image.open("ady_img.jpg").resize((1200, 800))
bg_photo = ImageTk.PhotoImage(bg_image)
bg_label = tk.Label(root, image=bg_photo)
bg_label.place(relwidth=1, relheight=1)

# Frame for Controls
control_frame = tk.Frame(root, bg='#3B4252', bd=2, relief=tk.RIDGE)
control_frame.place(x=800, y=50, width=350, height=700)

# Title Label
title_label = tk.Label(control_frame, text="Robot Control Panel", font=title_font, fg='white', bg='#3B4252')
title_label.pack(pady=10)

# TCP Position Label
tcp_label = tk.Label(control_frame, text="TCP Position (mm):\n[0.00, 0.00, 0.00]", font=label_font, fg='white', bg='#3B4252')
tcp_label.pack(pady=10)

# Joint Positions Label
joints_label = tk.Label(control_frame, text="Joint Positions (deg):\n[0.00, 0.00, 0.00, 0.00, 0.00, 0.00]", font=label_font, fg='white', bg='#3B4252')
joints_label.pack(pady=10)

# Movement Buttons Frame
move_frame = tk.Frame(control_frame, bg='#3B4252')
move_frame.pack(pady=10)

# Movement Buttons
move_x_pos = tk.Button(move_frame, text="Move +X", command=lambda: move_robot_x(0.02), font=button_font, bg='#4C566A', fg='white')
move_x_neg = tk.Button(move_frame, text="Move -X", command=lambda: move_robot_x(-0.02), font=button_font, bg='#4C566A', fg='white')
move_y_pos = tk.Button(move_frame, text="Move +Y", command=lambda: move_robot_y(0.02), font=button_font, bg='#4C566A', fg='white')
move_y_neg = tk.Button(move_frame, text="Move -Y", command=lambda: move_robot_y(-0.02), font=button_font, bg='#4C566A', fg='white')
move_z_pos = tk.Button(move_frame, text="Move +Z", command=lambda: move_robot_z(0.02), font=button_font, bg='#4C566A', fg='white')
move_z_neg = tk.Button(move_frame, text="Move -Z", command=lambda: move_robot_z(-0.02), font=button_font, bg='#4C566A', fg='white')

move_x_pos.grid(row=0, column=1, padx=5, pady=5)
move_x_neg.grid(row=0, column=0, padx=5, pady=5)
move_y_pos.grid(row=1, column=1, padx=5, pady=5)
move_y_neg.grid(row=1, column=0, padx=5, pady=5)
move_z_pos.grid(row=2, column=1, padx=5, pady=5)
move_z_neg.grid(row=2, column=0, padx=5, pady=5)

# Show/Hide Movement Controls
move_frame.pack_forget()  # Start hidden
def toggle_movement():
    if move_frame.winfo_ismapped():
        move_frame.pack_forget()
    else:
        move_frame.pack()

move_toggle_btn = tk.Button(control_frame, text="Show/Hide Movement", command=toggle_movement, font=button_font, bg='#5E81AC', fg='white')
move_toggle_btn.pack(pady=10)

# Depth Overlay Toggle
show_depth_var = tk.BooleanVar()
show_depth_check = tk.Checkbutton(control_frame, text="Show Depth Points", var=show_depth_var, command=lambda: toggle_depth(), font=label_font, fg='white', bg='#3B4252', selectcolor='#4C566A')
show_depth_check.pack(pady=5)

def toggle_depth():
    global show_depth
    show_depth = show_depth_var.get()

# Face Detection Toggle
show_face_var = tk.BooleanVar()
show_face_check = tk.Checkbutton(control_frame, text="Show Face Detection", var=show_face_var, command=lambda: toggle_face(), font=label_font, fg='white', bg='#3B4252', selectcolor='#4C566A')
show_face_check.pack(pady=5)

def toggle_face():
    global show_face
    show_face = show_face_var.get()

# 3x3 Grid Toggle
show_grid_var = tk.BooleanVar()
show_grid_check = tk.Checkbutton(control_frame, text="Show 3x3 Depth Grid", var=show_grid_var, command=lambda: toggle_grid(), font=label_font, fg='white', bg='#3B4252', selectcolor='#4C566A')
show_grid_check.pack(pady=5)

def toggle_grid():
    global show_grid
    show_grid = show_grid_var.get()

# Stream Frame
stream_label = tk.Label(root, bg='black')
stream_label.place(x=50, y=50, width=700, height=500)

# Function to move robot along Z, X, Y axis
def move_robot_z(z_move):
    displacement = f"p[0.00, 0.00, {z_move}, 0.0, 0.0, 0.0]"
    command = f"movel(pose_trans(get_actual_tcp_pose(), {displacement}), a=0.1, v=0.1)\n"
    send_robot_command(command)

def move_robot_x(x_move):
    displacement = f"p[{x_move}, 0.00, 0.00, 0.0, 0.0, 0.0]"
    command = f"movel(pose_trans(get_actual_tcp_pose(), {displacement}), a=0.1, v=0.1)\n"
    send_robot_command(command)

def move_robot_y(y_move):
    displacement = f"p[0.00, {y_move}, 0.00, 0.0, 0.0, 0.0]"
    command = f"movel(pose_trans(get_actual_tcp_pose(), {displacement}), a=0.1, v=0.1)\n"
    send_robot_command(command)

# 3x3 Grid for depth visualization
def draw_depth_grid(frame, depth_frame):
    ih, iw, _ = frame.shape
    grid_size = 3
    grid_step_x = iw // grid_size
    grid_step_y = ih // grid_size
    grid_depths = []

    # Draw the grid and calculate depth for each cell
    for i in range(grid_size):
        for j in range(grid_size):
            x = i * grid_step_x + grid_step_x // 2
            y = j * grid_step_y + grid_step_y // 2
            depth = depth_frame.get_distance(x, y)
            grid_depths.append(depth)

            # Draw a circle in the grid and display depth value
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"{depth:.2f}", (x + 10, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    return frame, grid_depths

# Function to update TCP and joint positions
def update_robot_data():
    try:
        if robot:
            # Get TCP pose
            pose = robot.get_pose()
            position = pose.pos.array  # [x, y, z] in meters
            orientation = pose.orient.array  # Rotation matrix

            # Get joint positions in degrees
            joints = robot.getj()
            joints_deg = [np.degrees(j) for j in joints]

            # Update labels
            tcp_label.config(text=f"TCP Position (mm):\n{np.round(position, 3)}\n\nRotation Matrix:\n{np.round(orientation, 3)}")
            joints_label.config(text=f"Joint Positions (deg):\n{np.round(joints_deg, 2)}")
    except Exception as e:
        print("Error updating robot data:", e)
    root.after(1000, update_robot_data)

# Start updating robot data
update_robot_data()

# Stop Stream Button
def stop_stream():
    global streaming
    streaming = False
    pipeline.stop()
    robot_socket.close()
    root.quit()

stop_button = tk.Button(root, text="Stop Stream", command=stop_stream, font=button_font, bg='red', fg='white')
stop_button.place(x=50, y=570)

# Start Streaming Thread
stream_thread = threading.Thread(target=stream_video, daemon=True)
stream_thread.start()

# Key Binding
root.bind("q", lambda event: stop_stream())

# Run Tkinter
root.mainloop()