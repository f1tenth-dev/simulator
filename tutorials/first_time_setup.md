## Getting Started (Basic Tutorials)

**Note**: If you have not yet completed the steps to install the simulator, the steps shown in this tutorial will not work. It is very important that you follow the instruction to download, compile, install and source the simulator as described.

The ROS F1/10 Autonomous Racecar Simulator consists of two major elements: the racecar and the race track. More details about the elements are provided throughout this tutorial.

**Racecar**:  
The racecar element of the simulator is the mobile, controllable, high-speed Ackermann-steering car-like robot that emulates the performance and dynamics of the [F1/10 platform](f1tenth.org). In the simulator, the racecar is represented as a collection of sensor, actuator and controller elements each described in a [xacro](http://wiki.ros.org/xacro), and whose visual and collision properties are set using a [urdf](http://wiki.ros.org/urdf). The racecar's power-train controls the rotation of its wheels and the angular displacement of its steering column and the controller parameters overseeing their performance is set in `config/control.yaml`  

The sensor suite currently tested and supported in the simulator include a 2D scanning lidar, a collision detection sensor, multiple RGB cameras and a wheel odometer. The sensor descriptions are usually loaded in the `xacro` description of the racecar and can be enabled/disabled by commenting in/out the sensor blocks in the file located at `urdf/macros.xacro`  

The simulator allows you to spawn the racecar with some or all the sensors by editing the file above. The user can also set the initial pose and certain control and visual properties of the racecar including:
1. Color of the racecar
2. Initial pose of the racecar `(x,y,z,theta)`
3. Remote tele-operation `enable` or `disable`

**Race Track**:  
The race track has two elements: the racing environment: `course` and the supervisory state exchange and monitoring node: `supervisor`. The course is the geometric wireframe that delineates the broundaries of the race track from the racing surface and also act as the visual rendering of the race track in the simulator's GUI. The simulator release includes a challenging race track that was generated using CAD and exported as an STL to be used by Gazebo. More information about the race track design and supplementing the race track can be found in advanced tutorials.  

The supervisor is a set of ROS nodes that form the active elements of the simulator seperate from the racecars. The supervisors measure lap times, race positions and relay agent state information that can be made avialable during dynamic or head-to-head autonomous racing. More features about the supervisor can be found in the tutorials section.

## First Time Setup
The race track course may sometimes not be installed by `catkin_make` though the files are already present in the correct directory. To overcome this situation, the course will have to be manually transferred from the `autosim_ws` workspace to the `~/.gazebo` workspace. To do this open a new terminal and enter the following command:

```console
user@ros-computer: cp -r autosim_ws/src/simulator/world/race_track .gazebo/models/
```

The above step needs to be completed only once, and the Gazebo simulator will be able to read the course information everytime the F1/10 simulator is launched. The user can now enter the following command in the terminal to launch the simulator for the first time:

```console
user@ros-computer: roslaunch f1tenth-sim simulator.launch run_gazebo:=true
```

Notice the information on the terminal; if there are any errors during installation, they would be shown in a red font. Not all errors on the terminal are fatal and not all fatal errors are necessarily displayed on the terminal. The Gazebo window should eventually come up on the screen and you should be able to see the simulated environment. Use the mouse to navigate and get yourself familiar with the simulator.  

In order to proceed to the next part of the tutorial press `ctrl + c` to exit the current session.

## Keyboard Tele-Operation
Keyboard teleoperation is the process of controlling the movement of the racecar using using computer's keyboard. The simulator release contains two teleoperation nodes meant to be ued with either a joystick game controller or the keyboard. This tutorial will focus only on the latter. For convenience, the keyboard teleoperation follows the conventional gaming control pattern `W-A-S-D` where W and S control forward and reverse velocities and A and D control steering position from left to right. The teleoperation node latches onto the control state of the car during the last recorded key stroke and makes this control state persist until a new command is recieved.

To bring up the simulator with the keyboard teleoperation node, kill any other simulation or Gazebo session that is active and enter the following command in a new terminal:

```console
user@ros-computer: roslaunch f1tenth-sim simulator.launch run_gazebo:=true keyboard_control:=true
```

Wait for the simulator session to be launched and the Gazebo GUI to appear. The terminal from which the session was launch needs to be the active terminal and placed on top of the Gazebo GUI in order to accept the commands from the keyboard. Use the W-A-S-D keys to navigate the racecar around the racetrack. Use this opportunity to make yourself familiar with the handling of the racecar.
