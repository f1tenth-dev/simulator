## Intermediate Tutorials
The set of tutorials in this section describe raing algorithms and how to spawn multiple racecars for dynamic or head-to-head racing. It is very important that you have understood the content in the previous tutorial section before proceeding with this section. This is especially true while dealing with coordinate frames for multiple racecars and high speed navigation as described in this tutorial section.

**Resolving tf-links and Namespaces**:  
If more than once racecar is active on the race track, all racecars share some reference frames while also possesing their unique frames. Keeping track of all these frames are necessary so that control commands are not passed to the wrong car. Every racecar must also have a unique namespace that identifies nodes and topics. For example, the command topic is usually set to `/command`, but if multiple racecars exist, they must be published under the individual racecar's namespace like `/car_1/command`, `/car_2/command` and `/car_N/command`. The same is true for nodes; for example, if the node controlling the car is `/purepursuit_node`, it must also be resolved using the same namespace as the topics leading to `/car_X/purepursuit_node` etc.  

Frame links on the other hand cannot handle the namespace resolution using `/`, so we use the format `<namespace>_<link_name>` to achieve full frame independence, for example `car_1` has a `base_link` set to `car_1_base_link` instead of `car_1/base_link`. It is important to note, however, global reference frames do not need to be resolved under individual namespaces. These frames are the `map` frame, the `odom` frame and the `world` frame. And example of the tf-tree for three racecars is shown in the image below:  

## Pure-Pursuit Racing Algorithm
Pure-Pursuit path tracking algorithm has proven itself to be highly capable and robust for high speed autonomous racing, and is our algorithm of choice for this simulator and the F1/10 physical platform. More information about the pure-pursuit algorithm can be found in this [paper](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf). The algorithm requires the odometry information of the racecar and a global racing line and works by generating a relative heading to a waypoint within the raceline which is a distance `l_d` away from the racecar on the raceline. This distance is the single tunable property of the pure-pursuit algorithm called the lookahead distance. The maximum achievalbe velocity of the racecar is proportional to the lookahead distance as a general rule of thumb. The paper linked above talks in general about the shortcomings of the pure-pursuit controller, the most significant of which is the tendency to cut corners (and potentially collide with the race track) if the lookahead distance is not properly tuned.  

To overcome the limitations of the pure-pursuit controller as described in the paper, and to make it a true autonomous racing algorithm, we have introduced some changes to the algorithm by using a supervisory node to adapt the lookahead distance based on the properties of the track at each waypoint along the raceline. We call this modification the Adaptive-Lookahead Pure-Pursuit. For this tutorial, we will use one of the modified racelines which contain information about lookahead distance at each waypoint to demonstrate the performance of the pure-pursuit controller and why we prefer this controller for autonomous racing. Kill all existing ROS nodes and the simulator (if running) and open a new terminal to enter the following command:

```console
user@ros-computer: roslaunch f1tenth-sim purepursuit_one_car.launch
```

The *rviz* window created uses a pre-configured setup for the pure-pursuit controller. The elements to note within the window are the raceline which is composed of different segments denoted using different colors which correspond to unique lookahead distances assigned to that segment of the raceline. The racecar will achieve the highest velocity at green segments and lowest at the red segments. It is interesting to note that the racecar in this tutorial is travelling twice as fast in its slowest setting compared to the highest speed achieved using the T.E.B. planner and ROS navigation described in the previous tutorial section.

## Multiple Autononomous Racecars
One of the salient features of the F1/10 simulator is the ability to spawn multiple racecars, and this feature can be used at the start of the simulation if you are interested in dynamic or head-to-head racing, or spawned at random during a current simulation session if you want to spawn racecars anywhere in the race track to test obstacle avoidance etc. Both of these capabilities are described in detail in this tutorial.

A launch file is the quickest way to start the simulator with multiple racecars. The simulator is provided with a `vehicle_class.launch.xml` file that can be imported into the main launch file for every racecar. The name assigned to the racecar becomes the namespace under which all nodes and packages for the said racecar is created. Along with unique names for each racecar, the user must specify the spawn location within the racetrack. For simplification, the simulator's origin is at the center of the race track, while the start/finish line is at `(-2.5m, 7.0m)` away from the center. The template to use in the launch file is shown below:

```xml
<include     file           = '$(find f1tenth-sim)/launch/simulator.launch'>
  <arg         name           = 'car_name'
               value          = '$(arg car_name)'/>
  <arg         name           = 'paint'
               value          = '$(arg car_paint)'/>
  <arg         name           = 'run_gazebo'
               value          = '$(arg run_gazebo)'/>
  <arg         name           = 'x_pos'
               value          = '$(arg x_pos)'/>
  <arg         name           = 'y_pos'
               value          = '$(arg y_pos)'/> </include>
```

In the template, you will notice the parameter `run_gazebo` which has to be set to `true` only for the first racecar and must be set to `false` for all subsequent racecars otherwise the simulator GUI will be forced to spawn multipe sessions ultimately leading to a crash. While this will not damage the simulator or the local machine, you might run into memory issues and have to restart the machine. An example for multiple autonomous cars is provided with the simulator; open a new terminal and enter the following command:

```console
user@ros-computer: roslaunch f1tenth-sim multi-TEB.launch
```

The simulator should have spawned multiple racecars each running a TEB based navigation stack. Use the `set_goal` command in the `rviz` window to set a goal and you will notice all racecars moving to the goal. We encourage you to repeat the above steps and have multiple racecars use pure-pursuit tracking to follow the raceline described in the previous tutorial. The following video shows multiple autonomous racecars using adaptive pure-pursuit for high speed racing in the simulator.
