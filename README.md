```markdown
# Unified Dashboard with Eclipse Ditto, ROS 2, and Virtual Studio

This guide provides a step-by-step process to create a unified dashboard in Virtual Studio for monitoring data from various sources, including IoT devices and production machines, using Eclipse Ditto, ROS 2, and The Things Network (TTN).

## Core Infrastructure Setup

### Phase 1: Setting up Eclipse Ditto

**Install and Configure Eclipse Ditto:** (Source: [Eclipse Ditto README](https://github.com/eclipse-ditto/ditto/README.md))

1.  **Clone Eclipse Ditto:**
    ```bash
    git clone [https://github.com/eclipse-ditto/ditto.git](https://github.com/eclipse-ditto/ditto.git)
    ```
2.  **Configure Ditto with Docker Compose:**
    ```bash
    cd ditto/deployment/docker/
    docker-compose up -d
    ```
3.  **Verify Ditto is running:**
    ```bash
    docker-compose logs -f
    ```

    You should now have the following running:
    * MongoDB (backing datastore)
    * Ditto microservices:
        * Policies
        * Things
        * Things-Search
        * Gateway
        * Connectivity

### Phase 2: Connecting to ROS 2 and Visualizing Data

**Install and Configure ROS 2:**

1.  **Install ROS 2 Jazzy:**
    * This demo uses ROS 2 Jazzy running on Ubuntu 24.04 Noble.
2.  **Add sourcing to your shell startup script:**
    ```bash
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    ```

**Install and Configure `ditto_ros_bridge`:** (Source: [ditto\_ros\_bridge README](https://github.com/virtualaichina/ditto_ros_bridge/README.md))

1.  **Clone and build the `ditto_ros_bridge` package:**
    ```bash
    git clone [https://github.com/virtualaichina/ditto_ros_bridge](https://github.com/virtualaichina/ditto_ros_bridge)
    cd ditto_ros_bridge
    colcon build
    source install/setup.bash
    sudo apt install python3-aiohttp
    ros2 launch ditto_ros_bridge bridge.launch.py
    ```
    Now, the `ditto_ros_bridge` is running.

**Install and Configure `rosbridge_server`:**

1.  **Install the `rosbridge_server` package:**
    ```bash
    sudo apt install ros-jazzy-rosbridge-server
    ```
2.  **Run the `rosbridge_server`:** (Ensure you are in the `ditto_ros_bridge` directory and have sourced `install/setup.bash`)
    ```bash
    cd ditto_ros_bridge/
    source install/setup.bash
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    ```
    The Rosbridge WebSocket server is now started on port 9090.

**Set Up Virtual Studio:**

1.  **Install and launch Virtual Studio:**
    ```bash
    sudo docker pull docker.io/virtualaichina/virtual-studio:v1.0
    sudo docker run -p "8002:8002" virtualaichina/virtual-studio:v1.0
    ```
    Open your web browser and navigate to `http://localhost:8002`.
2.  **Connect Virtual Studio to `rosbridge_server`:**
    * Click the Virtual Studio icon (menu) -> Open connection.
    * In the popup dialog, choose Rosbridge.
    * Enter `ws://localhost:9090`.

## Industrial Manufacturing Simulation and Testing

**Simulate industrial manufacturing data using `smart_manufacturing_sim.py`:**

1.  **Navigate to the `ditto_ros_bridge` directory:**
    ```bash
    cd ditto_ros_bridge/
    ```
2.  **Run the simulation script:**
    ```bash
    python3 smart_manufacturing_sim.py
    ```

**Verify ROS topics in Virtual Studio:**

* Click "Topics" in the left panel of Virtual Studio.
* You should see the industrial manufacturing ROS topics.

    ![Ros Topics](ros_topics.JPG)

**Add panels to monitor data:**

* Add raw message panels or other suitable panels to Virtual Studio.
* Listen to the topics you want to monitor.
* The simulation script updates industrial data in Eclipse Ditto, which is reflected in your panels.

    ![Panels](panels.JPG)
```
