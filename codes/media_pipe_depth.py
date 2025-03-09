import pyrealsense2 as rs
import cv2
import mediapipe as mp
import numpy as np

# Initialize MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

while True:
    # Get frames from RealSense
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        continue

    # Convert color frame to RGB for MediaPipe
    rgb_image = cv2.cvtColor(np.asanyarray(color_frame.get_data()), cv2.COLOR_BGR2RGB)

    # Process the frame to detect human pose
    results = pose.process(rgb_image)

    # Create a blank image for drawing pose
    blank_image = np.zeros((480, 640, 3), dtype=np.uint8)

    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark
        h, w, _ = blank_image.shape

        for landmark in mp_pose.PoseLandmark:
            x, y = int(landmarks[landmark].x * w), int(landmarks[landmark].y * h)

            # Ensure x, y are within bounds of the image size
            x = max(0, min(x, w - 1))
            y = max(0, min(y, h - 1))

            # Get depth value at this landmark
            depth = depth_frame.get_distance(x, y)

            # Print depth for each landmark
            print(f"Landmark {landmark}: Depth = {depth} meters")

            # Draw the landmark and depth value
            cv2.circle(blank_image, (x, y), 5, (0, 255, 255), -1)  # Neon Yellow
            cv2.putText(blank_image, f"{landmark}: {depth:.2f}m", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # Show the output with depth
    cv2.imshow("MediaPipe Pose with Depth", blank_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the RealSense pipeline and close windows
pipeline.stop()
cv2.destroyAllWindows()
