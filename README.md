# Setup
## Robot
1. Since this project was implemented using a TurtleBot3, it is highly recommended
to refer to the quick start guide on the TurtleBot3 e-manual alongside the following
steps [e-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/).
2. Install ROS2 Humble on Robot using the [installation documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) or the [e-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
3. Make sure the [Turtlebot3 package](https://github.com/ROBOTIS-GIT/turtlebot3) is installed if you are
planning to use turtlebots, as they contain the necessary "urdf" files for burger and
waffle turtlebots, as well as necessary packages for generating maps.
4. Ensure environment variables are set up - this includes the ROS domain ”ROS DOMAIN”,
the robot model (used to to check robot dimensions for navigation) and the
”ROBOT NAMESPACE”, which is the namespace you would like the robot to be
under.
5. Download the ”state pubsub” and ”multi bringup” packages from the project
[GitHub repository](https://github.com/marguhrita/RobotWarehousePackages).
6. Create a ”turtlebot3 ws” directory, and create a ”src” directory within. Move
the ”state pubsub” and ”multi bringup” packages into the ”src” directory. The
directories can be created using the following commands.
```bash
mkdir turtlebot3_ws
cd turtlebot3_ws
mkdir src
```
7. Ensure that there is a ”yaml” file which corresponds to the robot model you are
using within the ”params” directory in the ”multi bringup” package - the project
comes with parameter files for ”burger” and ”waffle” turtlebots.
8. Within the ”yaml” files, ensure that the root key is the namespace of the robot.
9. Run the following command while in the ”turtlebot3 ws” directory.

```bash
colcon build --symlink-install
```

10. Source the installation script which should be generated in a new ”install” direc-
tory within ”turtlebot ws” with the following command. This can be added to the
”bashrc” file to avoid needing to manually execute it in future runs.

```bash
source ˜/turtlebot3_ws/install/setup.bash
```

11. The robot initialization can now be run with the following command
```bash
ros2 launch multi_bringup multi_bringup.launch.py
```

## Central Commander
1. This installation is highly recommended on a device running the[ Ubuntu](file:///C:/Users/Alastair/Downloads/Robot_Warehouse_Report-4.pdf#cite.ubuntu-website) 22.04
Operating System.
2. Install the ”robot w”, ”state pubsub” and ”RobotWarehouse” packages from the
[GitHub repository](file:///C:/Users/Alastair/Downloads/Robot_Warehouse_Report-4.pdf#cite.robotwarehouse-github).
3. Create a ”turtlebot3 ws” directory, with a ”src” directory inside it.

```bash
mkdir turtlebot3_ws
cd turtlebot3_ws
mkdir src
```

4. Move the ”robot w” and ”state pubsub” packages into the ”src” directory.
5. Install ”cartographer”, ”teleop” and ”turtlebot bringup” packages for map genera-
tion - this can be done by following the ”Quick Start” section in the [Turtlebot3 e-manual](file:///C:/Users/Alastair/Downloads/Robot_Warehouse_Report-4.pdf#cite.turtlebot3_emanual).
6. Run the following command while in the ”turtlebot3 ws” directory.

```bash
colcon build --symlink-install
```

7. Follow the ”SLAM” instructions on the e-manual, or view the ”Map Saving”
section in Appendix B B.3 to navigate the robot around your chosen area, and
save the map to the ”RobotWarehouse” package as ”map.yaml” and ”map.pgm”.
8. Navigate into the ”RobotWarehouse” package and run ”main.py” to start the system. If Python is not installed, it will need to be installed.

```bash
sudo apt install python3
cd RobotWarehouse
python3 main.py
```

9. Any initialized robots on the same ROS domain as the controller will show up on
the user interface!

## Saving a Map
These instructions are adapted from the Turtlebot3 e-manual.
1. Initialize the robot without namespacing (on the robot) by running the following command.
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

2. Run cartographer (SLAM Algorithm) and teleop (movement)
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
ros2 run turtlebot3_teleop teleop_keyboard
```

3. Use the ”WASDX” keys to navigate the robot around your area until you are
satisfied with the generated map.

4. Save the map by running the following command - the final argument is the name
of your map.
```bash
ros2 run nav2_map_server map_saver_cli -f ˜/map
```

5. Make sure to move the generated map files into the ”RobotWarehouse” package!

## Simulation

To run the simulation, perform these additional steps
1. Move ”turtlebot multi robot” into ”turtlebot ws/src” directory.
2. Make sure Gazebo is installed - check the [e-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) for instructions
3. run colcon build within ”turtlebot ws”.
```bash
colcon build --symlink-install
```
4. Run the following command to start a simulation with two robots
```bash
ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_world.launch.py enable
```
5. Run ”main.py” in ”RobotWarehouse”