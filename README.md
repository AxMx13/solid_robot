# Example open source URDF robot for MoveIt2

### How to use:

Clone this repository to desired ros2 workspace:
```
mkdir -p ~/solid_robot_ws/src
cd ~/solid_robot_ws/src
git clone ...
cd ..
colcon build
. install/setup.bash
```
<p>To test our URDF, you can use the <code>ros-humble-urdf-tutorial</code> package by installing it with the following command:</p>

```
sudo apt install ros-humble-urdf-tutorial
ros2 launch urdf_tutorial display.launch.py model:=$HOME/solid_robot_ws/src/solid_robot/urdf/solid_robot.urdf.xacro
```