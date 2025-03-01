A step by step guide to create a unified dashboard in Virtual Studio to monitor data from various sources (IoT devices, production machines, etc.) using Eclipse Ditto, ROS 2, and The Things Network (TTN).

Core Infrastructure setup:

    Phase 1: Setting up the Eclipse Ditto

        Install and Configure Eclipse Ditto: (source: https://github.com/eclipse-ditto/ditto/README.md)
            Step 1.1: clone Eclipse Ditto to your server or cloud environment. Open a new terminal,
                `git clone https://github.com/eclipse-ditto/ditto.git`
            Step 1.2: Configure Ditto to create digital twins of your assets.
                `cd ditto/deployment/docker/`
                `docker-compose up -d`
            Step 1.3: Verify Ditto is running and accessible.
                `docker-compose logs -f`

        You are now running:
            a MongoDB as backing datastore of Ditto (not part of Ditto but started via Docker)
            Ditto microservices:
                Policies
                Things
                Things-Search
                Gateway
                Connectivity


    Phase 2: Connecting to ROS 2 and Visualizing Data

        Install and Configure ROS 2:
            Step 3.1: Install ROS 2 on your system.
                This demo uses ros2 jazzy running on Ubuntu 24 Noble.
            Step 3.2: Add sourcing to your shell startup script.
                `echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc`

        Install and Configure ditto_ros_bridge: (source:https://github.com/virtualaichina/ditto_ros_bridge/README.md)
            Step 4.1: Clone and build the ditto_ros_bridge package. Open a new terminal,
                `git clone https://github.com/virtualaichina/ditto_ros_bridge`
                `cd ditto_ros_bridge`
                `colcon build`
                `source install/setup.bash` 
                `sudo apt install python3-aiohttp`
                `ros2 launch ditto_ros_bridge bridge.launch.py`
            
            Now, the ditto-ros-bridge is running.

        Install and Configure ros_bridge:
            Step 5.1: Install the ros_bridge package.
                `sudo apt install ros-jazzy-rosbridge-server`
            Step 5.2: Run the bridge to connect ROS 2 to Virtual Studio. (Make sure you are in the ditto_ros_bridge directory and source install/setup.bash, otherwise it won't recognize the message types specified in the ditto_ros_bridge package.) Open a new terminal,

            `cd ditto_ros_bridge/`
            `source install/setup.bash`
            `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
            
            Now, the Rosbridge WebSocket server is started on port 9090


        Set Up Virtual Studio:
            Step 6.1: Install and launch Virtual Studio.
                `sudo docker pull docker.io/virtualaichina/virtual-studio:v1.0`

                `sudo docker run -p "8002:8002" virtualaichina/virtual-studio:v1.0`

                Now open your web browser and type in the URL: http://localhost:8002, welcome to Virtual Studio!
            
            Step 6.2: Connect Virtual Studio to ros_bridge.
                click on menu (Virtual Studio icon) -> Open connection, in the popup dialog choose Rosbridge, then type in: `ws://localhost:9090`

            Now that everything is set up, let's test it!


Industrial manufacturing simulation and Testing:

    Simulate industrial manufacturing data using smart_manufacturing_sim.py script:
        Open a new terminal,  
        `cd ditto_ros_bridge/`
        run the script: 
        `python3 smart_manufacturing_sim.py`

    if everything works properly, when you click on "Topics" at the left panel of your Virtual Studio, you should see all the ros topics on Industrial manufacturing available:
    ![Ros Topics](ros_topics.JPG)

    Now you can add raw message panels or other suitable panels of Virtual Studio to listen to the topics you want to monitor, the simulation script will update the industrial data at Eclipse Ditto periodically, which will then be reflected to your panels for monitoring :D
    ![Panels](panels.JPG)
