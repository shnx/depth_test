import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Configure the pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline
pipeline.start(config)

start_time = time.time()

while True:
    # Wait for a frame
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        print("Missing frame, skipping this iteration...")
        continue

    # Convert to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())

    # Get image dimensions
    h, w, _ = color_image.shape

    # Define 9 points in a 3x3 grid
    grid_points = [
        (w // 4, h // 4), (w // 2, h // 4), (3 * w // 4, h // 4),
        (w // 4, h // 2), (w // 2, h // 2), (3 * w // 4, h // 2),
        (w // 4, 3 * h // 4), (w // 2, 3 * h // 4), (3 * w // 4, 3 * h // 4)
    ]

    # Get depth at each grid point and overlay on the image
    for (x, y) in grid_points:
        depth = depth_frame.get_distance(x, y)
        cv2.circle(color_image, (x, y), 5, (0, 255, 0), -1)  # Draw a small circle
        cv2.putText(color_image, f"{depth:.2f}m", (x - 30, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the color image with depth information
    cv2.imshow('RealSense Depth Grid', color_image)

    # Handle the exit after 10 seconds or when 'q' is pressed
    if time.time() - start_time > 10:
        print("Displaying for 10 seconds, exiting...")
        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the pipeline and close any windows
pipeline.stop()
cv2.destroyAllWindows()
