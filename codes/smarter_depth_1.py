import socket
import numpy as np
import pyrealsense2 as rs
import cv2
from time import sleep, time

# Connect to the robot
robot_ip = "192.168.2.104"
robot_port = 30002
robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
robot_socket.connect((robot_ip, robot_port))
robot_socket.setblocking(False)  # Avoid blocking waits

# Function to send robot commands
def send_robot_command(command):
    robot_socket.send(command.encode())

# Set robot TCP and payload
send_robot_command("set_tcp([0, 0, 0.1, 0, 0, 0])\n")
send_robot_command("set_payload(0.1, [0, 0, 0.1])\n")
sleep(0.2)  # Allow time for processing

# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Initialize OpenCV face detection (DNN-based, much faster)
net = cv2.dnn.readNetFromCaffe(
    cv2.data.haarcascades + "deploy.prototxt",
    cv2.data.haarcascades + "res10_300x300_ssd_iter_140000_fp16.caffemodel"
)

def get_face_depth(color_image, depth_frame):
    """Detect faces and return depth."""
    h, w = color_image.shape[:2]
    blob = cv2.dnn.blobFromImage(color_image, 1.0, (300, 300), (104, 177, 123))
    net.setInput(blob)
    detections = net.forward()

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.6:  # Confidence threshold
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (x, y, x2, y2) = box.astype("int")
            depth = depth_frame.get_distance((x + x2) // 2, (y + y2) // 2)
            if 0.3 <= depth <= 1.2:  # Valid depth range
                return depth, (x, y, x2 - x, y2 - y)
    return None, None

# Main loop
frame_skip = 2  # Process every 2nd frame
frame_count = 0

while True:
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        continue

    frame_count += 1
    if frame_count % frame_skip != 0:
        continue  # Skip frames for speed boost

    color_image = np.asanyarray(color_frame.get_data())
    depth, face_position = get_face_depth(color_image, depth_frame)

    if depth is not None:
        x, y, w, h = face_position
        cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(color_image, f"Depth: {depth:.2f}m", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if depth < 0.5:
            z_move = -np.clip(0.5 - depth, -0.02, 0.02)
        elif depth > 0.6:
            z_move = -np.clip(0.6 - depth, -0.02, 0.02)
        else:
            z_move = 0  # No movement needed

        if z_move:
            command = f"movel(pose_trans(get_actual_tcp_pose(), p[0.00, 0.00, {z_move}, 0.0, 0.0, 0.0]), a=0.2, v=0.2)\n"
            send_robot_command(command)

    cv2.imshow("Face Detection", color_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

robot_socket.close()
pipeline.stop()
cv2.destroyAllWindows()
