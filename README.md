## Installation and Setup

The simulator has been tested on a variety of hardware configurations, and while Gazebo and ROS dependencies work with bare minimum hardware requirements, the essence of an autonomous racing simulator requires a real-time factor of at-least 0.85. For this reason, we suggest that your local machine meet the requirements stated in the following table.

|                  | Suggested Minimum Requirements |
|:----------------:|:------------------------------:|
| Operating System |  Ubuntu 18.04 (Bionic) 64-bit  |
|      Memory      |           16 GiB DDR4          |
|        GPU       |         NVIDIA GTX-1660        |

The simulator itself depends on libraries, packages and objects from ROS and Gazebo. The current version of the simulator uses ROS Melodic and Gazebo 9; along with certain other libraries and their python bindings. Before proceeding with the installation steps in this document, check if your local machine already has the full-desktop version of ROS Melodic installed. If not, follow the instructions provided in this [link](http://wiki.ros.org/melodic/Installation/Ubuntu). When you get to section 1.4 of the ROS installation tutorial, choose the first option: `ros-melodic-desktop-full`. Your local machine will now have the basic dependencies installed. The next steps of the installation tutorial are critical and must be followed in the shown sequence.

First, we begin by installing the ROS and Gazebo packages that are not installed by default using the above method. This includes SLAM packages, ROS navigation and the MIT GPU particle filter. Some of these packages have a Debian installation candidates and the others have to be downloaded as source codes and locally compiled. We install the Debian packages first; open a new terminal and enter the following command:

```console
user@ros-computer: sudo apt-get install -y ros-melodic-navigation ros-melodic-teb-local-planner* ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros-control ros-melodic-ackermann-msgs ros-melodic-serial qt4-default
```

Once the above installation process is completed, the simulator source along with the particle filter and mapping package will have to be downloaded. To do this, we first need to create a ROS workspace. Open a new terminal and enter the following commands in the given sequence:

```console
user@ros-computer: mkdir -p autosim_ws/src
user@ros-computer: cd autosim_ws/src
user@ros-computer: catkin_init_workspace
```

The simulator workspace is now initialized. The source for the packages will now have to be placed in the `src` directory to be compiled later. In the same terminal, enter the following commands to pull the sources:

```console
user@ros-computer: git clone https://github.com/f1tenth-dev/simulator
user@ros-computer: git clone https://github.com/mit-racecar/particle_filter
user@ros-computer: git clone https://github.com/kctess5/range_libc
user@ros-computer: git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam
```

The source for the required packages, including the simulator, are now in the workspace. The simulator depends on the GPU particle filter developed by the MIT RACECAR team and this package has to be configured before compiling the ROS packages. Navigate to the `range_libc` folder to compile the library necessary for the particle filter. Open a new terminal and enter the following commands:

```console
user@ros-computer: sudo pip install cython
user@ros-computer: cd autosim_ws/src/range_libc/pywrapper
```

There are two methods for installing the particle filter; with GPU support and without GPU support. We recommend using the package with GPU support, but leave the decision to you.

**Installing with GPU Support**  
Keep this terminal open and pay special attention to the next step. The particle filter package depends heavily on the GPU and its architecture, so it becomes very important to match the architecture of the GPU in your local machine to the one listed in the configuration of `setup.py`. We are particularly interested in the `sm_xx` value associated with the GPU and more information can be found in this [article](https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/) which is a good reference for understanding types NVIDIA's GPU. Once the architecture type has been identified, go back to the terminal and open the setup file using an editor of your choice and navigate to line 96:

```python
nvcc_flags = ['-arch=sm_20', '--ptxas-options=-v', '-c', '--compiler-options', "'-fPIC'", "-w","-std=c++11"]
```

Replace `-arch=sm_20` with the `-arch=sm_xx` value from the article in the link provided above. Once you have made the changes, save the file and exit back to the terminal and enter the following command to compile the library and follow the instruction on the screen:

```console
user@ros-computer: ./compile_with_cuda.sh
```

**Installing without GPU Support**  
In the same terminal, enter the following command and follow the instructions in the screen:

```console
user@ros-computer: ./compile.sh
```

With the final dependency installed, the installation process can now be continued using ROS `catkin_make`. This process is relatively simple; open a new terminal and enter the following command:

```console
user@ros-computer: cd autosim_ws
user@ros-computer: catkin_make install
```

The installation process in now complete, and the workspace needs to be sourced. Sourcing the workspace permanently helps launching the simulator easier. To do this, the `.bashrc` file in the home directory has to be modified; open a new terminal and enter the following command:

```console
user@ros-computer: echo "source ~/autosim_ws/devel/setup.bash" >> ~/.bashrc
user@ros-computer: source ~/.bashrc
```

You have now completed the steps necessary to install the simulator. The tutorial section will help you get started with the simulator.

**Uninstalling the Simulator**  
Removing the simulator from your local machine is a two-step process: first remove the line from `~/.bashrc` that sources the simulator workspace and then delete the `autosim_ws` directory.
