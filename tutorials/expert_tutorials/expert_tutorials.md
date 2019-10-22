## Expert Tutorials

Congratulations on completing the previous tutorial sections! You are now ready to use the ROS F1/10 simulator to test your own algorithms. To make the simulator experience smoother, we are providing you with a guide for using your own race-track and setting the vehicle state on the go. Gazebo publishes topics in formats described under `gazebo_msgs` for setting the physics state of all elements in the simulator including position, velocity, torque and momentum etc. Publishing the correct information to these topics can be a useful way to reset individual elemnts in the simulator that would otherwise require a full simulator restart.

## Set Racecar State
For modifying the state of individual racecars in the simulator, we have provided a node described by `set_racecar_state.py` which publishes a `ModelState.msg` message to the Gazebo topic `/gazebo/set_model_state`. More information about this message is provided in this [link](http://docs.ros.org/jade/api/gazebo_msgs/html/msg/ModelState.html). Within the context of the simulator, we are interested in the followind fields of the message:

1. `model_name`: Referes to the name of the racecar
2. `reference_frame`: Same as the global `map` frame (This is assigned by default).

The node expects 2 additional arguments where each argument is a tuple. The first tuple is the desired positional information and the second tuple is the desired initial velocity after spawn. To test this capability, open a new terminal to launch the simulator:

```console
user@ros-computer: roslaunch f1tenth-sim simulator run_gazebo:=false
```

Observe the position of the racecar in the siulator GUI and launch a new terminal to enter the following command. The tuple contains 3 elements describing the information on each axis relative to the race track; for the first positional tuple, the structure is `x_pos y_pos z_pos` and for the second velocity tuple: `x_vel y_vel z_vel`. All data within the tuple must be a float for the node to work.

```console
user@ros-computer: rosrun f1tenth-sim set_racecar_state.py car_1 0.0 0.0 5.0 0.0 0.0 0.0
```

The first tuple commands the node to move the racecar to the origin of the race track and raise it to 5.0 meters above the surface of the race track. You can enter this command multiple times to get the same effect. If you wish to set the initial velocity of the racecar, modify the second tuple accordigly. It is important to note that the velocity tuple sets the velocity to the racecar as a physical object and no torque will be observeable on the wheels.

## Use Different Race Track
The race track provided with the simulator is a 3D mesh created using SOLIDWORKS first as a 2D sketch. Use this [tutorial](https://help.solidworks.com/2018/english/SolidWorks/sldworks/c_getting_started_drawings.htm?id=5af0e96173b345cfbbf93dd30d666123#Pg0) to create a 2D sketch if you are unfamiliar with CAD. It is important to have a closed loop in the sketch for racing algorithms to work, but if your intention is to just play around with the simulator, any type of track is suitable. Once you have the 2D sketch ready, it is time to extrude the part of the sketch that will eventually become the boundaries of the race track. This [tutorial](https://help.solidworks.com/2017/english/SolidWorks/sldworks/c_2D_to_3D_Conversion.htm) explains how to convert your 2D sketch to a 3D model. Once the sketch has been converted, save it as an STL file in your local machine. The STL file generated using this process becomes the mesh for the race track in the simulator.

The race track and the race environment are described in the `race_track.world` under the `world/` directory. Move the STL file to the race_track directory under the world directory and remember the name of the STL file. Edit the `race_track.world` file and navigate to the bottom of the file until you reach the following section:

```xml
<model name="race_track">
  <pose>-39.47 -22.7 0 0 0 0</pose>
  <static>true</static>
  <link name="body">
    <collision name="collision">
      <geometry>
        <mesh>
          <uri>model://race_track/race_track.stl</uri>
          <scale>0.133 0.133 0.05</scale>
        </mesh>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <mesh>
          <uri>model://race_track/race_track.stl</uri>
          <scale>0.133 0.133 0.025</scale>
        </mesh>
      </geometry>
    </visual>
  </link>
</model>
```

Replace the `race_track.stl` with the name of the 3D STL file you created. You may need to adjust the dimension scales, but if your file was generated using SOLIDWORKS, it should fit on the asphalt plane at the center of the simulator. For 3D files generated using other CAD software, open a pull request in the official simulator repository and we will help you to adjust the scales.
