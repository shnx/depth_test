import cv2
import pyrealsense2 as rs
import numpy as np
import socket
import time

# Universal Robot connection settings
UR_IP = "192.168.2.104"  # Change to your UR robot's IP
UR_PORT = 30002

# Initialize socket for sending URScript commands
ur_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ur_socket.connect((UR_IP, UR_PORT))

# Initialize RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Load OpenCV's Haar cascade face detector
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

# Control parameters
Kp_x, Kp_y, Kp_z = 0.002, 0.002, 0.05  # Tuning factors for movement speed

# Low movement speed to avoid breaking the robot
MOVEMENT_SPEED = 0.1  # Lower velocity and acceleration values

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect faces
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        if len(faces) > 0:
            print(f"Detected {len(faces)} faces.")  # Debug message: Number of faces detected
            # Pick the largest face detected
            faces = sorted(faces, key=lambda f: f[2] * f[3], reverse=True)
            (x, y, w, h) = faces[0]

            # Compute center of the face
            face_x, face_y = x + w // 2, y + h // 2
            depth = depth_frame.get_distance(face_x, face_y)

            # Calculate position offsets (assuming RealSense is in end effector center)
            h, w, _ = color_image.shape
            error_x = (face_x - w // 2)  # Left-right offset
            error_y = (face_y - h // 2)  # Up-down offset
            error_z = depth - 0.5  # Target depth (0.5m example)

            # Debug messages for calculated errors
            print(f"Face at position ({face_x}, {face_y}), depth: {depth:.2f}m")
            print(f"Error X: {error_x}, Error Y: {error_y}, Error Z: {error_z:.2f}")

            # Compute movement values
            move_x = -Kp_x * error_x  # Negative because UR follows standard frame
            move_y = Kp_y * error_y
            move_z = Kp_z * error_z

            # Debug message for calculated movement values
            print(f"Calculated movement (x: {move_x}, y: {move_y}, z: {move_z})")

            # Send movement command to UR only if face is detected
            ur_command = f"movel(pose_add(get_actual_tcp_pose(), p[{move_x}, {move_y}, {move_z}, 0, 0, 0]), a={MOVEMENT_SPEED}, v={MOVEMENT_SPEED})\n"
            ur_socket.send(ur_command.encode())

            # Draw face bounding box and depth info
            cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(color_image, f"Depth: {depth:.2f}m", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        else:
            print("No face detected.")  # Debug message when no face is found

        # Display result
        cv2.imshow("Face Tracking", color_image)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    pipeline.stop()
    ur_socket.close()
    cv2.destroyAllWindows()
