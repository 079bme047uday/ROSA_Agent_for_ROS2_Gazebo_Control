from langchain.agents import tool
from rosa import ROSA, RobotSystemPrompts
from llm_robot import azure_llm 
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import sys # Added for graceful exit

# -------------------------------
# ROS2 INIT
# -------------------------------
# Initialize ROS 2 only once at the start
rclpy.init() 
node = Node("gazebo_llm_controller")
vel_pub = node.create_publisher(Twist, "/cmd_vel", 10) 
print("ROS2 Node 'gazebo_llm_controller' started.")

# -------------------------------
# TOOL: MOVE FORWARD
# -------------------------------
@tool
def move_forward(distance: float) -> str:
    """Move the robot straight forward or backward by a given distance (meters). 
    Use a positive distance for forward, negative for backward."""
    
    linear_speed = 0.5  # Adjust this speed for your specific robot
    direction = 1.0 if distance >= 0 else -1.0
    
    speed = linear_speed * direction
    duration = abs(distance / linear_speed)

    vel = Twist()
    vel.linear.x = speed

    start = time.time()
    while time.time() - start < duration:
        vel_pub.publish(vel)
        rclpy.spin_once(node, timeout_sec=0.1) 

    vel_pub.publish(Twist())
    return f"Moved forward {distance} meters."

# -------------------------------
# TOOL: ROTATE
# -------------------------------
@tool
def rotate(angle_deg: float) -> str:
    """Rotate the robot by a specified angle (degrees). Positive is counter-clockwise."""
    
    angular_speed = 0.5 
    duration = abs(angle_deg * 0.0174533) / angular_speed 

    vel = Twist()
    vel.angular.z = angular_speed if angle_deg > 0 else -angular_speed

    start = time.time()
    while time.time() - start < duration:
        vel_pub.publish(vel)
        rclpy.spin_once(node, timeout_sec=0.5) 

    vel_pub.publish(Twist())
    return f"Rotated {angle_deg} degrees."

# -------------------------------
# TOOL: SET INITIAL POSE (Unchanged)
# -------------------------------
@tool
def set_initial_pose(x: float, y: float, yaw_deg: float = 0.0) -> str:
    """Sets the robot's initial position and orientation in the map."""
    
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "map"
    
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0 

    yaw_rad = yaw_deg * (3.14159 / 180.0) 
    q_z = math.sin(yaw_rad / 2) 
    q_w = math(yaw_rad / 2) 
    
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = q_z
    msg.pose.pose.orientation.w = q_w
    
    pub.publish(msg)
    time.sleep(1.0) 

    return f"Published initial pose to ({x}, {y}) facing {yaw_deg} degrees."

# -------------------------------
# ROSA SETUP (Unchanged)
# -------------------------------
prompts = RobotSystemPrompts(
    embodiment_and_persona="You control a Gazebo robot via ROS 2 velocity commands. You can move and rotate the robot."
)

tools = [
    move_forward,
    rotate,
    set_initial_pose
    
]

agent = ROSA(
    ros_version=2,
    llm=azure_llm,
    tools=tools,
    prompts=prompts
)

# -------------------------------
# INTERACTIVE COMMAND LOOP
# -------------------------------
def run_interactive_agent():
    """Continuously prompts the user for commands and executes them."""
    print("\n========================================================")
    print("   ROSA Gazebo Robot Controller Active")
    print("   Enter your commands in natural language (e.g., 'move 5m forward').")
    print("   Type 'quit' or 'exit' to stop the controller.")
    print("========================================================")

    while True:
        try:
            # Get command from the user
            user_input = input("\nRobot Command > ").strip()

            if user_input.lower() in ["quit", "exit"]:
                print("\nShutting down robot controller...")
                break
            
            if not user_input:
                continue

            # Invoke the agent with the user's command
            print("Thinking...")
            result = agent.invoke(user_input)
            
            # Print the result from the agent
            print(f"\nAgent Execution Result:\n{result}")

        except KeyboardInterrupt:
            # Handle Ctrl+C gracefully
            print("\nShutting down robot controller...")
            break
        except Exception as e:
            print(f"\nAn error occurred: {e}")
            break

    # Clean up ROS 2 resources
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

# Run the interactive environment
if __name__ == "__main__":
    run_interactive_agent()