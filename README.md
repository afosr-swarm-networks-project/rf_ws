# AFOSR ROS2

## Usage
Clone the repo
```bash
git clone https://github.com/afosr-swarm-networks-project/rf_ws.git
cd rf_ws
```
We have docker container for both the agent and the host, to use them, you need to install docker on both sides first.
After docker is installed, you can run the following commands to run the containers.
### Jetson on dog (192.168.1.51)

MAKE SURE YOU ARE USING CYCLONEDDS TO AVOID A WEIRD RAM ISSUE!

Set the environment variable ROBOT_NUM to something like 1,2,3,4 so it can be discovered by the viewer. IT defaults to 1.


```
mkdir -p ~/unitree_ws/src
cd ~/unitree_ws/src
git clone --recurse-submodules https://github.com/snt-arg/unitree_ros.git
cd ~/unitree_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch minimal_startup unitree_go1_minimal_startup_launch.py
```

In a new terminal:
```
colcon build --packages-select rf_control rf_msgs
source ~/rf_ws/install/setup.bash
ros2 run rf_control controller_node
```

In a new terminal:
```
docker compose up rf_agent
```


It takes time to build the docker image the first time you run the command, make sure you have internet connection.

### Host Side
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
# in the repo root:
colcon build --packages-select rf_msgs rf_visualization
rqt
# go to plugins -> visualization -> RF waterfall viewer
```

Once you see that the topics are coming through properly, run the following from repo root: 

```
# set DISPLAY=:0 if you are remoting into the host machine
# (activate your venv first)
cd src/cartography/rf_lab/cartography/engine
python3 step_viewer.py
```

And watch the magic! 
