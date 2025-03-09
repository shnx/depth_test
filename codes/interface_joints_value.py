from pyrobot import Robot

def get_joint_values():
    try:
        # Create robot instance (change to the appropriate robot you are using)
        robot = Robot('ur5')  # Change 'ur5' to your robot model

        # Get joint positions
        joint_values = robot.arm.get_joint_angles()

        print("Joint Values:", joint_values)
    except Exception as e:
        print(f"Error: {e}")

# Get joint values
get_joint_values()
