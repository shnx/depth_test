import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import time

# Robot IP address
ROBOT_IP = "192.168.2.104"  # Updated with your robot's IP address

# Configuration file for RTDE
CONFIG_FILE = "rtde_config.xml"

# Load configuration
conf = rtde_config.ConfigFile(CONFIG_FILE)
output_names, output_types = conf.get_recipe("out")
input_names, input_types = conf.get_recipe("in")

# Connect to the robot
con = rtde.RTDE(ROBOT_IP, 30004)
con.connect()

# Setup recipes
con.send_output_setup(output_names, output_types)
input_setup = con.send_input_setup(input_names, input_types)

# Get current robot pose
if not con.send_start():
    print("Failed to start data synchronization")
    exit()

# Read current pose
current_pose = con.receive()
target_pose = current_pose.actual_TCP_pose

# Move 10 cm in the z+ direction
target_pose[2] += 0.10  # z+ direction (in meters)

# Send the new target pose to the robot
input_setup.input_double_register_0 = target_pose[0]
input_setup.input_double_register_1 = target_pose[1]
input_setup.input_double_register_2 = target_pose[2]
input_setup.input_double_register_3 = target_pose[3]
input_setup.input_double_register_4 = target_pose[4]
input_setup.input_double_register_5 = target_pose[5]

# Execute the move
con.send(input_setup)

# Wait for the move to complete
time.sleep(2)  # Adjust the sleep time as needed

# Stop data synchronization
con.send_pause()

# Disconnect from the robot
con.disconnect()