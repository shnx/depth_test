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

# Start streaming
pipeline.start(config)

while True:
    # Get frames from RealSense
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        continue

    # Convert color frame to numpy array
    color_image = np.asanyarray(color_frame.get_data())

    # Convert to RGB for MediaPipe
    rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

    # Process the frame to detect human pose
    results = pose.process(rgb_image)

    if results.pose_landmarks:
        for landmark in results.pose_landmarks.landmark:
            h, w, _ = color_image.shape
            x, y = int(landmark.x * w), int(landmark.y * h)

            # Ensure x, y are within the valid range
            x = max(0, min(x, w - 1))
            y = max(0, min(y, h - 1))

            # Get depth at the keypoint location
            depth = depth_frame.get_distance(x, y)

            # Draw the landmark
            cv2.circle(color_image, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(color_image, f"{depth:.2f}m", (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # Show output
    cv2.imshow("Skeleton Detection with RealSense D415", color_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
pipeline.stop()
cv2.destroyAllWindows()
