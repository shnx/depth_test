import tkinter as tk
import threading
import socket
import numpy as np
import pyrealsense2 as rs
import cv2
import mediapipe as mp
from time import sleep

# Connect to the robot via socket
robot_ip = "192.168.2.104"
robot_port = 30002
robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
robot_socket.connect((robot_ip, robot_port))

# Function to send commands to the robot
def send_robot_command(command):
    robot_socket.send(command.encode())
    print(f"Sent command: {command}")

# Set the robot TCP and payload
send_robot_command("set_tcp([0, 0, 0.1, 0, 0, 0])\n")
send_robot_command("set_payload(2, [0, 0, 0.1])\n")
sleep(0.2)  # Allow some time for the robot to process the setup commands

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

# GUI window for robot movement control
def create_gui():
    root = tk.Tk()
    root.title("Robot Movement Control")

    # Buttons for Z-axis movement
    button_up = tk.Button(root, text="Move +Z", command=lambda: move_robot_z(0.02))
    button_up.pack(pady=10)

    button_down = tk.Button(root, text="Move -Z", command=lambda: move_robot_z(-0.02))
    button_down.pack(pady=10)

    # Buttons for X-axis movement
    button_x_plus = tk.Button(root, text="Move +X", command=lambda: move_robot_x(0.02))
    button_x_plus.pack(pady=10)

    button_x_minus = tk.Button(root, text="Move -X", command=lambda: move_robot_x(-0.02))
    button_x_minus.pack(pady=10)

    # Buttons for Y-axis movement
    button_y_plus = tk.Button(root, text="Move +Y", command=lambda: move_robot_y(0.02))
    button_y_plus.pack(pady=10)

    button_y_minus = tk.Button(root, text="Move -Y", command=lambda: move_robot_y(-0.02))
    button_y_minus.pack(pady=10)

    root.mainloop()

# Run GUI in a separate thread
def run_gui():
    gui_thread = threading.Thread(target=create_gui)
    gui_thread.daemon = True  # Ensure it exits when the program ends
    gui_thread.start()

# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Ensure color stream is enabled
pipeline.start(config)

# Initialize MediaPipe Face Detection
mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.2)

# Function to get depth to the detected face
def get_face_depth(frame, depth_frame, bbox):
    x, y, w, h = bbox
    # Get the depth of the center of the detected face
    depth = depth_frame.get_distance(x + w // 2, y + h // 2)
    return depth

# Function to calculate distance from bounding box to frame edges in pixels
def calculate_pixel_distances(frame, bbox):
    ih, iw, _ = frame.shape
    x, y, w, h = bbox
    
    # Calculate distances from bounding box to the edges of the frame
    left_dist = x
    right_dist = iw - (x + w)
    top_dist = y
    bottom_dist = ih - (y + h)
    
    return left_dist, right_dist, top_dist, bottom_dist

# Main loop to track the face and adjust the robot's position
def main_loop():
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

        # Apply color map to depth image for better visualization
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Convert to RGB for MediaPipe processing
        rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        results = face_detection.process(rgb_image)

        # Create a blank image for visualization
        annotated_image = color_image.copy()

        if results.detections:
            for detection in results.detections:
                # Get bounding box of the face
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, _ = color_image.shape
                x, y, w, h = int(bboxC.xmin * iw), int(bboxC.ymin * ih), int(bboxC.width * iw), int(bboxC.height * ih)

                # Draw bounding box around the face
                cv2.rectangle(annotated_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Get the depth to the detected face
                depth = get_face_depth(color_image, depth_frame, (x, y, w, h))

                if depth is not None:
                    print(f"Detected face at depth: {depth} meters")

                    # If the face is within 60 cm (0.6 meters), move the robot closer or farther
                    if depth < 0.5:
                        print("Face is too close, moving robot away.")
                        move_robot_z(0.02)  # Move away
                    elif depth > 0.6:
                        print("Face is too far, moving robot closer.")
                        move_robot_z(-0.02)  # Move closer
                    else:
                        move_robot_z(0)  # No movement if within the desired range
                else:
                    print("No face detected.")
                
                # Calculate and display pixel distances from the bounding box to frame edges
                left_dist, right_dist, top_dist, bottom_dist = calculate_pixel_distances(color_image, (x, y, w, h))

                # Show the distances in the top-left corner of the image
                font_scale = 1.0
                thickness = 2
                text = [
                    f"Left Distance: {left_dist} px",
                    f"Right Distance: {right_dist} px",
                    f"Top Distance: {top_dist} px",
                    f"Bottom Distance: {bottom_dist} px"
                ]

                # Add the text to the image
                y_offset = 30
                for line in text:
                    cv2.putText(annotated_image, line, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness)
                    y_offset += 30  # Move down for each line of text

        # Show the annotated image
        cv2.imshow('Face Detection and Robot Control', annotated_image)
        cv2.imshow('Depth Image', depth_colormap)

        # Sleep for a short time to allow for real-time movement
        sleep(0.1)

        # Exit loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Close the socket connection
    robot_socket.close()
    pipeline.stop()
    cv2.destroyAllWindows()

# Run GUI in a separate thread and the main loop
if __name__ == "__main__":
    run_gui()  # Start the GUI in a separate thread
    main_loop()  # Run the main robot control loop
