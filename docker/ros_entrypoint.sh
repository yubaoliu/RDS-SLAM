#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/melodic/setup.bash"
source "/root/catkin_ws/devel/setup.bash"

# Build MaskRCNN
cd /root/catkin_ws/src/MaskRCNN_ROS/include/MaskRCNN
python setup.py install

echo "================ RDS-SLAM Ready =============="

cd /root/catkin_ws

exec "$@"
