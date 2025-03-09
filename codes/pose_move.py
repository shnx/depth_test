import socket

# Connect to the robot
robot_ip = "192.168.2.104"
robot_port = 30002
robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
robot_socket.connect((robot_ip, robot_port))

# Define the displacement vector in the TCP's local frame
# This moves the TCP 10 mm along its Z-axis
displacement = "p[0.00, 0.00, 0.0, -0.0, 0.0, 0.0]"

# Create the URScript command to move the TCP
command = f"movel(pose_trans(get_actual_tcp_pose(), {displacement}), a=0.2, v=1.0)\n"

# Send the command to the robot
robot_socket.send(command.encode())

print("Robot moving +10mm in Z direction relative to the TCP frame.")
robot_socket.close()
