import tkinter as tk
from tkinter import ttk
import urx
import math
import pyrealsense2 as rs

# Function to convert radians to degrees
def rad_to_deg(rad):
    return rad * 180 / math.pi

# Function to get joint positions in degrees
def get_joint_positions(robot):
    joint_positions = robot.getj()
    joint_positions_deg = [rad_to_deg(joint) for joint in joint_positions]
    return joint_positions_deg

# Function to get TCP pose as a matrix
def get_tcp_pose(robot):
    try:
        pose = robot.get_pose()  # Use get_pose() to get the Transform object
        position = pose.pos  # Extract position (x, y, z) from the Transform object
        orientation = pose.rotation  # Extract orientation (rotation matrix)
        tcp_matrix = f"Orientation:\n{orientation}\nPosition:\n{position}"
        return tcp_matrix
    except Exception as e:
        print(f"Error getting TCP pose: {e}")
        return "Error retrieving TCP pose"

# Initialize robot connection
robot = urx.Robot("192.168.2.104")

# Set up the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)

# Tkinter Interface Setup
root = tk.Tk()
root.title("Robot Control Interface")

# Joint Positions in Degrees Display
def update_joint_positions():
    joint_positions_deg = get_joint_positions(robot)
    joint_positions_label.config(text="Joint Positions (Degrees):\n" + str(joint_positions_deg))
    root.after(1000, update_joint_positions)

# TCP Pose Display
def update_tcp_pose():
    tcp_matrix = get_tcp_pose(robot)
    tcp_pose_label.config(text=f"Current TCP Pose:\n{tcp_matrix}")
    root.after(1000, update_tcp_pose)

# Intel RealSense Depth Points Display
def update_depth_points():
    # Wait for a new set of frames
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        return
    # Get the depth points for a 3x3 area in the middle of the frame
    width = depth_frame.get_width()
    height = depth_frame.get_height()
    depth_points = []
    for i in range(3):
        row = []
        for j in range(3):
            depth = depth_frame.get_distance(width // 2 + i - 1, height // 2 + j - 1)
            row.append(round(depth, 3))  # round the depth to 3 decimal places
        depth_points.append(row)
    
    # Display the depth points in the GUI
    depth_points_text = "\n".join(["\t".join(map(str, row)) for row in depth_points])
    depth_points_label.config(text="Intel RealSense Depth Points (3x3):\n" + depth_points_text)
    root.after(1000, update_depth_points)

# Move Robot Functionality
def move_robot(axis, direction):
    joint_positions = robot.getj()
    if axis == "x":
        joint_positions[0] += 0.1 * direction  # Move 0.1 rad along x-axis
    elif axis == "y":
        joint_positions[1] += 0.1 * direction  # Move 0.1 rad along y-axis
    elif axis == "z":
        joint_positions[2] += 0.1 * direction  # Move 0.1 rad along z-axis
    robot.movej(joint_positions, 1.0)  # Use movej instead of setj

# Create the 'Show Controls' button to toggle visibility of movement buttons
def toggle_controls():
    if move_controls_frame.winfo_ismapped():
        move_controls_frame.grid_forget()
    else:
        move_controls_frame.grid(row=3, column=0, columnspan=2)

# Buttons for movement (only shown when requested)
move_controls_frame = ttk.Frame(root)

move_x_pos_button = tk.Button(move_controls_frame, text="Move +X", command=lambda: move_robot("x", 1))
move_x_pos_button.grid(row=0, column=0)

move_x_neg_button = tk.Button(move_controls_frame, text="Move -X", command=lambda: move_robot("x", -1))
move_x_neg_button.grid(row=0, column=1)

move_y_pos_button = tk.Button(move_controls_frame, text="Move +Y", command=lambda: move_robot("y", 1))
move_y_pos_button.grid(row=1, column=0)

move_y_neg_button = tk.Button(move_controls_frame, text="Move -Y", command=lambda: move_robot("y", -1))
move_y_neg_button.grid(row=1, column=1)

move_z_pos_button = tk.Button(move_controls_frame, text="Move +Z", command=lambda: move_robot("z", 1))
move_z_pos_button.grid(row=2, column=0)

move_z_neg_button = tk.Button(move_controls_frame, text="Move -Z", command=lambda: move_robot("z", -1))
move_z_neg_button.grid(row=2, column=1)

# Labels to show joint positions, TCP pose, and depth points
joint_positions_label = ttk.Label(root, text="Joint Positions (Degrees):\n")
joint_positions_label.grid(row=0, column=0, columnspan=2)

tcp_pose_label = ttk.Label(root, text="Current TCP Pose:\n")
tcp_pose_label.grid(row=1, column=0, columnspan=2)

depth_points_label = ttk.Label(root, text="Intel RealSense Depth Points (3x3):\n")
depth_points_label.grid(row=2, column=0, columnspan=2)

# 'Show Controls' button to toggle the movement control buttons
show_controls_button = tk.Button(root, text="Show Movement Controls", command=toggle_controls)
show_controls_button.grid(row=4, column=0, columnspan=2)

# Start the update loops
update_joint_positions()
update_tcp_pose()
update_depth_points()

# Start the Tkinter main loop
root.mainloop()

# Cleanup
pipeline.stop()
robot.close()
