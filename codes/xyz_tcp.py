import socket
import struct
import numpy as np
import time
# Robot IP and port
robot_ip = "192.168.2.104"
robot_port = 30002
robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
robot_socket.connect((robot_ip, robot_port))

# Function to send commands to the robot
def send_robot_command(command):
    robot_socket.send(command.encode())

# Function to get TCP pose from the robot (Tool Frame)
def get_tcp_position_tool_frame():
    try:
        send_robot_command("get_actual_tcp_pose()")  # Get TCP pose in tool frame
        tcp_pose = robot_socket.recv(1024)
        
        if tcp_pose:
            if len(tcp_pose) >= 24:
                tcp_values = struct.unpack('6f', tcp_pose[:24])
                return tcp_values  # (X, Y, Z, Rx, Ry, Rz)
            else:
                print("Invalid data received.")
        else:
            print("No data received.")
    except Exception as e:
        print(f"Error: {e}")

# Function to apply transformation (assumes you have R and T)
def transform_to_base_frame(tcp_values, R, T):
    # Extract TCP position and orientation
    x_tool, y_tool, z_tool, rx, ry, rz = tcp_values

    # Create position vector in tool frame
    tcp_pos_tool = np.array([x_tool, y_tool, z_tool])
    
    # Apply transformation: Position in base frame = R * Position in tool frame + T
    tcp_pos_base = np.dot(R, tcp_pos_tool) + T
    print(f"TCP Position (Base Frame): {tcp_pos_base}")
    
    return tcp_pos_base

# Example Transformation Matrix (R and T), adjust as needed
R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # Identity rotation (no rotation)
T = np.array([0.1, 0.2, 0.3])  # Example translation (offset from base to TCP)

# Main loop to get TCP position with respect to base
while True:
    tcp_values = get_tcp_position_tool_frame()
    if tcp_values:
        tcp_pos_base = transform_to_base_frame(tcp_values, R, T)
    time.sleep(1)

# Close the socket connection
robot_socket.close()
