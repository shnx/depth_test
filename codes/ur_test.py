import socket

UR_IP = "192.168.2.104"  # Change to your robot's IP
UR_PORT = 30002  # Universal Robots default port

ur_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ur_socket.connect((UR_IP, UR_PORT))
print("Connected to UR!")

# Send a simple command
ur_socket.send(b"set_digital_out(0, False)\n")  # Example: Turn on digital output 0
ur_socket.close()
