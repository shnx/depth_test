import cv2
import pyrealsense2 as rs
import numpy as np
import socket

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable color and depth streams
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start the pipeline
pipeline.start(config)

# Load the pre-trained face detection model from OpenCV
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Universal Robot IP and Port
robot_ip = "192.168.2.104"
robot_port = 30002
robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
robot_socket.connect((robot_ip, robot_port))

# Function to send move command to robot
def move_robot_slowly(x_move, y_move, z_move):
    # Send small move command to the robot in mm (slow movement)
    command = f"movej([ {x_move}, {y_move}, {z_move}, 0, 0, 0], a=0.02, v=0.02)\n"
    robot_socket.send(command.encode())

# Function to display the decision to move the robot
def display_movement_decision(image, move_x, move_y, depth):
    cv2.putText(image, f"Suggested move - X: {move_x:.2f}mm, Y: {move_y:.2f}mm", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    cv2.putText(image, f"Depth: {depth:.2f}m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    cv2.putText(image, "Press 'y' to Move, 'n' to Cancel", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

while True:
    # Wait for frames from RealSense
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        continue

    # Convert the color frame to a numpy array
    color_image = np.asanyarray(color_frame.get_data())

    # Convert to grayscale for face detection
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # Detect faces
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    # If a face is detected
    if len(faces) > 0:
        # Pick the first face (if multiple faces are detected)
        (x, y, w, h) = faces[0]

        # Calculate the center of the face
        face_center_x = x + w // 2
        face_center_y = y + h // 2

        # Get the depth of the face from the depth frame
        depth = depth_frame.get_distance(face_center_x, face_center_y)

        # Get the image dimensions (height, width)
        img_height, img_width, _ = color_image.shape

        # Calculate the offset from the center of the image
        offset_x = face_center_x - img_width // 2
        offset_y = face_center_y - img_height // 2

        # Print the depth and offset information
        print(f"Face detected at: X = {face_center_x}, Y = {face_center_y}")
        print(f"Depth (meters): {depth:.2f}")
        print(f"Offset from center - X: {offset_x} pixels, Y: {offset_y} pixels")

        # Convert pixel offsets to mm (simplified conversion based on depth)
        scale_x = 0.05  # mm per pixel (this is a simplification, requires calibration)
        scale_y = 0.05  # mm per pixel
        move_x = offset_x * scale_x * depth  # in mm
        move_y = offset_y * scale_y * depth  # in mm
        move_z = 0  # No movement in Z direction for simplicity

        # Apply small movement steps for slow motion
        move_x = np.clip(move_x, -2, 2)  # limit to 2 mm movement
        move_y = np.clip(move_y, -2, 2)  # limit to 2 mm movement

        # Display the decision on the image
        display_movement_decision(color_image, move_x, move_y, depth)

    # Show the image with the face detection and movement recommendation
    cv2.imshow("Face Detection with Depth", color_image)

    # Wait for user input
    key = cv2.waitKey(1) & 0xFF

    if key == ord('y'):  # 'y' to send the move command
        move_robot_slowly(move_x, move_y, move_z)
        print("Movement command sent to robot.")
    elif key == ord('n'):  # 'n' to cancel
        print("Movement canceled.")
    
    # Exit if 'q' is pressed
    if key == ord('q'):
        break

# Stop the pipeline and release resources
pipeline.stop()
robot_socket.close()
cv2.destroyAllWindows()
