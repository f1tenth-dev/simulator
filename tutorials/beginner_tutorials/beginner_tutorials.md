## Basic Tutorials
This tutorial section introduces the user to the basic capabilities of the ROS F1/10 Autonomous Racing Simulator including: mapping, localization and autonomous navigation. But, before we start the tutorial, it is important to understand the use of the [tf library](http://wiki.ros.org/tf) and how it is implemented in the simulator. The tf library is responsible for keeping track of the various reference frames and is at the foundation of autonomous navigation.

**tf-Implementation**:  
The ROS F1/10 simulator consists of a number of coordinate frames whose geometric relationships can be static or dynamic. Example of static tf-frames is LiDAR to racecar; where the LiDAR reports a list of distances that create a 2D point-cloud w.r.t itself but this list has to be converted to show the geometric relationship to the racecar instead. The LiDAR is at a constant pose w.r.t the racecar and as such requires a static transformation of sensor values. For this purpose, we use a static transform broadcaster as described in the snippet below:

```xml
<node        name           = '$base_laser_link'
             pkg            = 'tf'
             type           = 'static_transform_publisher'
             args           = '0.325 0.0 0.05
                               0.0 0.0 0.0
                               $(arg car_name)_base_link
                               $(arg car_name)_laser
                               20'/>
```

Within the `args` section, the first line is relevant to the example; stating that the LiDAR is 0.325 meters infront of and 0.05 meters above the racecar's `base_link` which is located at the rear-axle of the racecar in accordance with Ackermann dynamics. To learn more about static tf, read the official [wiki](http://wiki.ros.org/tf#static_transform_publisher).

**Dynamic-tf and Precision Odometry**:  
Dynamic frame transformations is necessary when the geometric relationship between two frames are continiously changing. An example of this type of `tf` relationship is between the `odom` and `base_link`. The `odom` frame is located at the position where the racecar is initialized and is critical for autonomous navigation. The odometry information and the dynamic transformation is handled by the `control_plugin` node that is unique to each racecar. The following snippet explains how this happens:

```python
def odom_callback(data):

    odom                      = Odometry()
    odom.header.frame_id      = odom_frame
    odom.child_frame_id       = base_frame
    odom.header.stamp         = rospy.Time.now()
    odom.pose                 = data.pose
    odom.pose.pose.position.x = odom.pose.pose.position.x - 2.5
    odom.pose.pose.position.y = odom.pose.pose.position.y + 7.0
    odom.twist = data.twist

    tf = TransformStamped(header         = Header(
                          frame_id       = odom.header.frame_id,
                          stamp          = odom.header.stamp),
                          child_frame_id = odom.child_frame_id,
                          transform      = Transform(
                          translation    = odom.pose.pose.position,
                          rotation       = odom.pose.pose.orientation))
```

The callback function `odom_callback` reads position and velocity data from Gazebo and modifies the position data with the planar offset between the `odom` frame and the geometric center of the race track. The information is also used by the inbuilt transform library found in the `tf2` package to publish the dynamic transformation.

**Tutorial Setup**:  
Each tutorial is included in a launch file that contains three elements: simulator, ROS nodes, and *rviz* visualization. These files can be found under `launch/` directory with the simulator package and can be modified if you so choose, but has to be used unmodified to be able to work with the tutorial.

## Simultaneous Localization and Mapping
The F1/10 simulator uses `hector_slam` package as the default ROS mapping package, although `gmapping` and `cartographer` have been tested. This tutorial is designed to be a walkthrough for mapping the race track in the simulator. The simulator is released with a map already installed, but if you are interested in learning how to create a map by yourself, you can do so by launching three terminals. One of the terminal will be used to launch the simulator with one racecar in teleoperation mode, the second will be used to launch the mapping nodes and the third will be used to store the map on your computer. You must complete the keyboard teleoperation tutorial to proceed with mapping. In the first terminal, enter the following command to launch the simulator:

```console
user@ros-computer: roslaunch f1tenth-sim simulator.launch run_gazebo:=true keyboard_control:=true
```

Wait for the simulator to start and do not move the racecar, as the initial position of the racecar will be used as the origin of the map. In the second terminal, enter the following command to bringup the mapping and related nodes:

```console
user@ros-computer: roslaunch f1tenth-sim mapping.launch
````

If everything works properly, you should see an *rviz* window open with the mapping configuration already loaded and the screen should show the initial occupancy grid color sequence. You may notice some warnings on the terminal that might display an interpolation/extrapolation error and this can be safely ignored. You are now using the `hector_mapping` package to create a 2D map of the race track! Bring the first terminal into focus to recieve the keyboard commands and drive the car around **slowly**. The `scanmather` node will skew the map if you are not careful in driving the racecar or if you drive too fast. Once you have completed one lap around the race track, you can save the map data published to the ROS `/map` topic as an image file. It is not recomended to overwrite the map already present in the`map/` directory as the existing map will be used over the next set of tutorials. In the third terminal, enter the following command to save the map locally:

```console
user@ros-computer: rosrun map_server map_saver -f <your_map_name>
```

The map will now be saved in your home directory as a set of two files: an image file showing the occupancy grid and a configuration file. You can use an image viewer of your choice to see the image of the map you have created.

## Prticle Filter Localization
Localization is the process of estimating the position of the racecar within the map using a probabilistic approach on data from sensors and odometer. Most ROS navigation packages use [Adaptive Monte Carlo Localization (AMCL)](http://wiki.ros.org/amcl). The F1/10 simulator uses the [Particle Filter](https://arxiv.org/abs/1705.01167) developed by the MIT-Racecar team which provides faster convergence and uses the computer's GPU (if available). Using the map generated as described in the previous tutorial, the localization node can be run using the following commands in a new terminal. The first command is to bring up the simulator in teleoperation mode:

```console
user@ros-computer: roslaunch f1tenth-sim simulator.launch run_gazebo:=true keyboard_control:=true
```

The second command in a new terminal will bring up the localization node with a map server and a pre-configured *rviz* visualizer. You can do this by entering the following command:

```console
user@ros-computer: roslaunch f1tenth-sim localization.launch
```

With the simulator launch terminal active, you can send commands to the racecar and view the *rviz* window to see the particle filter localization in action. You can move the racecar as slow or as fast as possible and the particle filter will be able to adjust the position of the racecar with relative ease.

## Autonomous Navigation using T.E.B. Planner
ROS navigation is a set of libraries that use a map to autonomously move the racecar from its current position to a goal position within the map. While generally applicable to all robots, the default ROS navigation is not well suited for robots with Ackermann-steering (car-like) geometry. For this reason, we chose the Timed Elastic Band (TEB) planner for navigation. Elements of ROS navigation like global plan and costmap are used by the local planner to produce time-optimal steering and velocity commands within the limits set by the user and/or the racecar. To make things easier, the simulator already contains a param file under `config/` that sets essential parameters of the navigation stack including parameters for costmap generation, robot footprint description and goal tolerances. The navigation tutorial can be rerun by modifying the parameters in the configuration file to see the effects of individual parameter on the performance of the racecar. For now, the parameter file contains the values that work best for the the current race track. To bring up the navigation stack, visualizer and the simulator all at once, use the following command in a new terminal:

```console
user@ros-computer: roslaunch f1tenth-sim navigation.launch
```

Wait for the simulator and the navigation nodes to come online. To autonomously move the racecar, use the mouse to select `2D Nav Goal` in the top bar of the *rviz* window and select a pose withing the map in the same window. If the goal is reachable, you should see the `global plan` generated in green and the T.E.B. `local plan` generated as a set of red arrows from the racecar's footprint. The racecar should now start moving towards the goal set by you. You can update the goal at any time should you choose to. The attached video shows the behaviour of the T.E.B. planner if all parameters are properly set.
