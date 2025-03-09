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

# Stick figure connection pairs (based on Mediapipe landmarks)
connections = [
    (mp_pose.PoseLandmark.LEFT_SHOULDER, mp_pose.PoseLandmark.RIGHT_SHOULDER),
    (mp_pose.PoseLandmark.LEFT_SHOULDER, mp_pose.PoseLandmark.LEFT_ELBOW),
    (mp_pose.PoseLandmark.LEFT_ELBOW, mp_pose.PoseLandmark.LEFT_WRIST),
    (mp_pose.PoseLandmark.RIGHT_SHOULDER, mp_pose.PoseLandmark.RIGHT_ELBOW),
    (mp_pose.PoseLandmark.RIGHT_ELBOW, mp_pose.PoseLandmark.RIGHT_WRIST),
    (mp_pose.PoseLandmark.LEFT_SHOULDER, mp_pose.PoseLandmark.LEFT_HIP),
    (mp_pose.PoseLandmark.RIGHT_SHOULDER, mp_pose.PoseLandmark.RIGHT_HIP),
    (mp_pose.PoseLandmark.LEFT_HIP, mp_pose.PoseLandmark.RIGHT_HIP),
    (mp_pose.PoseLandmark.LEFT_HIP, mp_pose.PoseLandmark.LEFT_KNEE),
    (mp_pose.PoseLandmark.LEFT_KNEE, mp_pose.PoseLandmark.LEFT_ANKLE),
    (mp_pose.PoseLandmark.RIGHT_HIP, mp_pose.PoseLandmark.RIGHT_KNEE),
    (mp_pose.PoseLandmark.RIGHT_KNEE, mp_pose.PoseLandmark.RIGHT_ANKLE)
]

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
        landmarks = results.pose_landmarks.landmark
        h, w, _ = color_image.shape

        # Convert landmarks to pixel coordinates
        pose_points = {}
        for landmark in mp_pose.PoseLandmark:
            x, y = int(landmarks[landmark].x * w), int(landmarks[landmark].y * h)
            x = max(0, min(x, w - 1))
            y = max(0, min(y, h - 1))
            pose_points[landmark] = (x, y)

        # Draw stick figure
        for connection in connections:
            p1, p2 = connection
            if p1 in pose_points and p2 in pose_points:
                cv2.line(color_image, pose_points[p1], pose_points[p2], (0, 255, 0), 3)

        # Draw joints
        for point in pose_points.values():
            cv2.circle(color_image, point, 5, (0, 0, 255), -1)

    # Show output
    cv2.imshow("Stick Figure Animation with RealSense", color_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
pipeline.stop()
cv2.destroyAllWindows()
