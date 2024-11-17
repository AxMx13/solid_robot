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
<p>To launch description use the following command:</p>

```
ros2 launch solid_robot display.launch.py 
```