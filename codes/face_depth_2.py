import socket
import numpy as np
import pyrealsense2 as rs
import cv2
from time import sleep

# Connect to the robot via socket
robot_ip = "192.168.2.104"
robot_port = 30002
robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
robot_socket.connect((robot_ip, robot_port))

# Function to send commands to the robot
def send_robot_command(command):
    robot_socket.send(command.encode())

# Set the robot TCP and payload
send_robot_command("set_tcp([0, 0, 0.1, 0, 0, 0])\n")
send_robot_command("set_payload(0.1, [0, 0, 0.1])\n")
sleep(0.2)  # Allow some time for the robot to process the setup commands

# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Ensure color stream is enabled
pipeline.start(config)

# Initialize OpenCV for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Function to get depth to the detected face
def get_face_depth(frame, depth_frame):
    # Convert the image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect faces in the image
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    
    # If no face is detected, return None
    if len(faces) == 0:
        return None, None
    
    # Get the coordinates of the first detected face (you can add logic for multiple faces if needed)
    (x, y, w, h) = faces[0]
    
    # Get the depth of the center of the detected face
    depth = depth_frame.get_distance(x + w // 2, y + h // 2)
    if depth > 1.2:
        return None, None

    
    return depth, (x, y, w, h)

# Main loop to track the face and adjust the robot's position
while True:
    # Wait for the next set of frames from RealSense
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    # Check if both depth and color frames are valid
    if not depth_frame or not color_frame:
        print("Error: Could not get depth or color frame.")
        continue

    # Convert the depth and color frames to OpenCV format
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    
    # Get the depth and face position
    depth, face_position = get_face_depth(color_image, depth_frame)
    
    if depth is not None:
        # Draw bounding box around the face
        (x, y, w, h) = face_position
        cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        # Show the distance to the face
        cv2.putText(color_image, f"Depth: {depth:.2f}m", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        # If the face is within 0.5 to 0.7 meters, we do nothing
        if 0.5 <= depth <= 0.6:
            print(f"Face detected at depth: {depth} meters (within range, no movement)")
        
        # If the face is closer than 0.5m, move the robot **away**
        elif depth < 0.5:
            print(f"Face detected at depth: {depth} meters (too close, moving away)")

            # Calculate displacement in the Z direction (move away if too close)
            z_offset = 0.5 - depth  # Positive to move away
            
            # Restrict the movement to 2 cm (0.02 meters) max per step
            z_move = -np.clip(z_offset, -0.02, 0.02)
            
            # Create the URScript command to move the robot in Z direction relative to its current position
            displacement = f"p[0.00, 0.00, {z_move}, 0.0, 0.0, 0.0]"
            command = f"movel(pose_trans(get_actual_tcp_pose(), {displacement}), a=0.1, v=0.1)\n"
            
            # Send the command to the robot
            send_robot_command(command)
            print(f"Moving robot by {z_move * 100} cm in Z direction")

        # If the face is farther than 0.7m, move the robot **closer**
        elif depth > 0.6:
            print(f"Face detected at depth: {depth} meters (too far, moving closer)")

            # Calculate displacement in the Z direction (move closer to the face)
            z_offset = 0.6 - depth  # Negative to move closer
            
            # Restrict the movement to 2 cm (0.02 meters) max per step
            z_move = -np.clip(z_offset, -0.02, 0.02)
            
            # Create the URScript command to move the robot in Z direction relative to its current position
            displacement = f"p[0.00, 0.00, {z_move}, 0.0, 0.0, 0.0]"
            command = f"movel(pose_trans(get_actual_tcp_pose(), {displacement}), a=0.1, v=0.1)\n"
            
            # Send the command to the robot
            send_robot_command(command)
            print(f"Moving robot by {z_move * 100} cm in Z direction")
        

        # Update the display window with the color image
        cv2.imshow("Face Detection", color_image)

    else:
        print("No face detected.")

    # Sleep for a short time to allow for real-time movement
    #sleep(0.1)
    
    # Ensure that the robot is still processing commands
    robot_socket.settimeout(0.1)  # Short timeout to check if the robot is still responding
    try:
        robot_socket.recv(1024)  # Try to read response from the robot
    except socket.timeout:
        # If no response, continue to send commands
        pass

    # Break the loop if the user presses 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Close the socket connection and stop the pipeline
robot_socket.close()
pipeline.stop()
cv2.destroyAllWindows()
