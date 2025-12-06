# ROSA_Agent_for_ROS2_Gazebo_Control
A natural language interface for controlling a mobile robot in Gazebo simulation. Uses LangChain/ROSA and Azure OpenAI to translate human commands (e.g., "move 3m forward") into ROS 2 velocity and service calls.

### 1. Prerequisites

Before running the agent, ensure you have the following installed and configured:

-   **ROS 2:** A working installation of ROS 2 (e.g., Humble or Jazzy).
-   **Gazebo Simulation:** A Gazebo environment launched with your mobile robot model.
-   **Python Environment:** Python 3.12 or above with pip.
-   **Environment Variables:** Your Azure OpenAI API credentials configured in a `.env` file (see below).

### 2. Installation

1.  **Clone the Repository:**
    ```bash
    git clone [https://github.com/079bme047uday/ROSA_Agent_for_ROS2_Gazebo_Control.git](https://github.com/079bme047uday/ROSA_Agent_for_ROS2_Gazebo_Control.git)
    cd ROSA_Agent_for_ROS2_Gazebo_Control
    ```

2.  **Install Python Dependencies(inside robot_llm directory):**
    ```bash
    #these are all dependencies from jpl-rosa library, inside a separate environment for robot_llm directory one can also do (pip install jpl-rosa) 
    pip install -r requirements.txt
    
    ```

3.  **Configure Azure OpenAI:**
    Create a file named **`.env`** in the robot_llm directory and add your credentials:
    ```
    AZURE_OPENAI_ENDPOINT="YOUR_ENDPOINT_URL"
    AZURE_OPENAI_API_KEY="YOUR_API_KEY"
    AZURE_DEPLOYMENT_NAME="gpt-4o" # Or your specific deployment name
    ```
### 3. Usage: Running the Interactive Agent

1.  **Start ROS 2 and Gazebo:**
    Ensure your Gazebo world is running and your robot is spawning its motion topics (especially `/cmd_vel`).

    ```bash
    # Example command (adjust for your specific robot launch file)
    ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
    ```

2.  **Execute the Agent Script:**
    Run the main Python script containing the interactive loop (run this from robot_llm directory):
    ```bash
    python test_robot.py
    ```

3.  **Enter Commands:**
    The controller will start and prompt you for input:

    ```
    ROSA Gazebo Robot Controller Active
    Robot Command >
    ```

### 4. Agent Tools Explained

The LLM is restricted to the following Python tools. Each tool directly publishes to a ROS 2 topic or service.

| Tool Name | ROS 2 Mechanism | Description |
| :--- | :--- | :--- |
| `move_forward` | Publishes `Twist` to **`/cmd_vel`** | Moves the robot using a **time-based kinematic approximation**. |
| `rotate` | Publishes `Twist` to **`/cmd_vel`** | Rotates the robot based on angular velocity over a duration. |
| `set_initial_pose` | Publishes `PoseWithCovarianceStamped` to **`/initialpose`** | Resets the robot's pose for localization systems (e.g., AMCL/Nav2). |


### 5. Advanced Features & Customization

- **Add New Robot Skills:**
  - Edit `test_robot.py` to add new @tool functions for your robot (e.g., sensor queries).
  - Register new tools in the `tools` list for the agent.

- **Change Topics/Parameters:**
  - Update topic names (e.g., `/cmd_vel`) and parameters in `test_robot.py` to match your robot.
  - Use ROS 2 parameters for easy launch-time configuration.

- **Switch LLM Provider:**
  - The agent uses Azure OpenAI by default. You can swap in OpenAI, local LLMs, or other providers by editing `llm_robot.py` and `.env`.

- **Run in Real Robot:**
  - The same agent can be used on a real robot if the topics and services match. Test in simulation first!

---

### 6. Troubleshooting

- **Package Not Found:**
  - Make sure you built and sourced your workspace:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select robot_llm
    source install/setup.bash
    ```
- **LLM API Errors:**
  - Check your `.env` file for correct API keys and endpoint.
  - Ensure your environment has internet access.
- **Robot Not Moving:**
  - Confirm Gazebo is running and `/cmd_vel` is being published.
  - Check for topic name mismatches.
- **Python Import Errors:**
  - Install missing dependencies with `pip install -r requirements.txt`.

---

### 7. Contributing & Extending

- Fork the repo and submit pull requests for new robot skills, LLM integrations, or bug fixes.
- See `test_robot.py` for examples of adding new @tool functions.
- For advanced LLM prompt engineering, see `llm_robot.py` and LangChain docs.

---

### 8. References

- [ROS 2 Documentation](https://docs.ros.org/en/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [LangChain](https://python.langchain.com/)
- [Azure OpenAI](https://learn.microsoft.com/en-us/azure/ai-services/openai/)
- [JPL ROSA Documentation](https://github.com/nasa-jpl/rosa)


###  A Note on Accuracy (Time-Based Control)

The `move_forward` and `rotate` tools rely on **time** to estimate distance and angle (e.g., time = distance / speed). **For professional or precise navigation, this method is insufficient.** A robust system would subscribe to the **`/odom`** (odometry) topic and use feedback to accurately measure distance traveled. This project uses the time-based method for simplicity and demonstration purposes.

