#!/bin/bash
source /opt/ros/humble/setup.bash
cd /home/rf_ws
export ROS_NAMESPACE=$(hostname)
echo ROS Namespace: $ROS_NAMESPACE
if [ ! -f src/rf_pipeline/resource/best.torchscript ]; then
  echo "TorchScript model not found, exporting..."
  yolo export model=src/rf_pipeline/resource/best.pt nms=True
  echo "TorchScript export complete."
fi
echo "Building workspace..."
colcon build --packages-ignore rf_visualization
source install/setup.bash
echo "Launching agent..."
ros2 launch rf_bringup agent.launch.py namespace:=$ROS_NAMESPACE

