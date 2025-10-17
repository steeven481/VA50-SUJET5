# ROS Scoutsan Docker

## Building the image

- Run the following command in a terminal
  `./build.sh`

- To force rebuilding the image use the option `-r`

## Running the container

- Launch the server by running the following command in a terminal
  `./server.sh`

- Launch as many client as you need by running the following command in a terminal
  `./client.sh`

- The folders `ros`, `workspace` and `share` are automatically mounted in the container, respectively as `/home/pal/.ros`, `/home/pal/ros_ws` and `/home/pal/share`
  This allows editing the workspace and looking through the logs from the development computer. Any file or folder copied to any of these folders will be accessible both from the container and the development computer.

- If the workspace is empty, i.e., only contains an empty folder `src`, run the following command in `/home/pal/ros_ws` inside the container to initialize it 
  `catkin_make -DCATKIN_ENABLE_TESTING=OFF -DDISABLE_PAL_FLAGS=ON`

## Working with external nodes

- If the ROS master is on a remote computer (e.g., when working with a real robot), set the `ROS_MASTER_URI` by running the following command inside the container (this must be repeated for each client)
  `export ROS_MASTER_URI=http://<MASTER_COMPUTER_IP>:11311`

- You must set `ROS_IP` to the address of the host computer on the same sub-network as the compute hosting the master.

- You may want to disable the firewall
  `sudo ufw disable`

## Read TIAGo handbook

Read the [TIAGo handbook](https://docs.pal-robotics.com/tiago-single/handbook.html) to familiarize yourself with the control of the simulated or real robot

## Working with TIAGo in simulation

- Execute the following commands inside the container
  `source /opt/pal/gallium/`
  `roslaunch tiago_198_gazebo tiago_gazebo.launch`

  This will launch a simulation with the robot in an empty world. If you want to use a existing world you can launch `tiago_mapping.launch` or `tiago_navigation.launch`. The first one will launch the robot in mapping mode, the second one in localization mode (both are in the same world).

- Simulated Hokuyo LIDARs may not work properly from inside the docker. If it is the case run the following command inside the container just before launching gazebo

  `export LIBGL_ALWAYS_SOFTWARE=1`

## Working with the real TIAGo

- Run the following command inside the container (this must be repeated for each client)
  `source ~/.bashrc tiago`
  
  This command will automatically set the `ROS_MASTER_URI` and  `ROS_IP` variables (assuming the development computer is connected to the Tiago in Wifi via the default hotspot, or directly via a ethernet cable)
  
- Connect to the TIAGo computer via ssh using the following command

  `ssh pal@10.68.0.1`

  the password is `pal`

- If the TIAGo computer cannot access the container you may want to disable the firewall on the development PC

  `sudo ufw disable`

## Using JetBrains Gateway to develop with CLion and PyCharm

- Install JetBrains Gateway and run it

- While the container is running, create a new SSH connection with
  - user: pal
  - host: localhost
  - port: 22

- Select `Check Connection and Continue`, password should be left empty

- Select the IDE version and click `Installation options...` to customize Installation path and put `/home/ros/share/JetBrains/RemoteDev/dist`
  Since `/home/ros/share` is a mounted directory, the installed IDEs will remain persistent and won't need to be downloaded each time the container is launched. Also, IDE settings will be kept.

- Enter the project directory `/home/pal/ros_ws/src`

- Click `Download IDE and Connect`

- With CLion you can customize the build path as explained here: https://www.jetbrains.com/help/clion/ros-setup-tutorial.html
# VA50-SUJET5
