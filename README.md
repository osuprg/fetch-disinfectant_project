# fetch-disinfectant-project

## Install

### Install dependencies
Need to install the rviz_visual_tools for the cone marker. Futher information [here](https://github.com/PickNikRobotics/rviz_visual_tools/blob/melodic-devel).
```
sudo apt-get update
sudo apt-get install ros-melodic-rviz-visual-tools
```

The octomap dependencies need to be installed.
```
sudo apt-get install ros-melodic-octomap
sudo apt-get install ros-melodic-octomap-server
sudo apt-get install ros-melodic-octomap-mapping
```

You also need to pip3 install:
* rospkg
* scipy
* sympy


### Build
Add the package to your src file in your workspace.

```
git clone https://github.com/osuprg/fetch_disinfectant_project.git
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

### Get started
Run launch files to get things started.

```
roslaunch fetch_project_moveit_config fetch_world.launch or roslaunch fetch_project_moveit_config fetch_world_collision.launch
roslaunch fetch_project_moveit_config disinfectant_project.launch
roslaunch fetch_project_moveit_config run_nodes.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
