# ROS2 / Aurora Robotics Lab Integration Code

The Aurora Robotics Lab control stack is not designed around ROS2, but for complex path planning tasks for object manipulation, we would like to use its path planners such as MoveIt. 

This integration is incomplete--a work in progress as of December 2024. 

## MoveIt Installation

First, be running Ubuntu 22.04, with ROS2 Humble. Installation guide:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

    sudo apt install ros-dev-tools ros-humble-ros-base ros-humble-desktop

Next, install MoveIt, following this tutorial to set up a workspace.  We describe `~/ws_moveit` as our workspace path below, to match the tutorial.

https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html

During the build step my 32GB machine hit the out of memory killer due to many parallel compiles, so I used:

    cd ~/ws_moveit/
    colcon build --parallel-workers 6

Finally, link over this folder (MiningRobot/ros2/src/lunatic_exchange) into your build workspace, and build our package:

    cd ~/ws_moveit/
    ln -s ~/MiningRobot/ros2/src/lunatic_exchange src/
    colcon build --parallel-workers 6 --packages-above  lunatic_exchange



## MoveIt Runtime

To set up a terminal to run MoveIt code, I needed:

    cd ~/ws_moveit/
    source /opt/ros/humble/setup.bash
    source install/setup.bash


## Excahauler in RViz

The ROS2 builtin urdf_tutorial package can display the excahauler model and let you move the joints with:

    ros2 launch urdf_tutorial display.launch.py model:=`pwd`/src/lunatic_exchange/config/excahauler.urdf



