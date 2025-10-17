
export QT_X11_NO_MITSHM=1
export VISP_INPUT_IMAGE_PATH=${HOME}/visp-ws/visp-images

source /opt/pal/gallium/setup.bash

source /opt/ros/${ROS_DISTRO}/setup.bash --extend

if [ -f ${ROS_WORKSPACE}/devel/setup.bash ]; then
  source ${ROS_WORKSPACE}/devel/setup.bash --extend
fi

export DISABLE_ROS1_EOL_WARNINGS=1

if [ "$1" = tiago ]; then
  echo 'ROS Configuration: Tiago as remote HOST'
  export ROS_IP=$(ip route get 10.68.0.1 | awk '{print $7; exit}')
  export ROS_MASTER_URI="http://10.68.0.1:11311/"
else
  echo 'ROS Configuration: localhost'
  export ROS_IP=127.0.0.1;
  export ROS_MASTER_URI="http://localhost:11311/"
fi

export GAZEBO_MODEL_PATH=/home/pal/ros_ws/src/va50/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=/home/pal/ros_ws/src/va50:$GAZEBO_RESOURCE_PATH
