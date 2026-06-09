# AFOSR ROS2

## Usage
Clone the repo
```bash
git clone https://github.com/afosr-swarm-networks-project/rf_ws.git
cd rf_ws
```
We have docker container for both the agent and the host, to use them, you need to install docker on both sides first.
After docker is installed, you can run the following commands to run the containers.
### Agent Side
```bash
docker composite up rf_agent
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
