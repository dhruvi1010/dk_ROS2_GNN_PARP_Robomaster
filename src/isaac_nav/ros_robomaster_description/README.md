# ros_robomaster_description

## Update: Launch robomaster description

To launch robomaster description (robot_state_publisher) just run: 

    ros2 launch ros_robomaster_description ros_robomaster_description.launch.py


To launch the description with a namespace run (it will also use the namespace as a tf prefix):

    ros2 launch ros_robomaster_description ros_robomaster_description.launch.py namespace:=<namespace>


This Package contrains the ros_robomaster_description XACRO Files.  The description is following the ROS [REP 145](https://ros.org/reps/rep-0145.html) and [REP 103](https://www.ros.org/reps/rep-0103.html#axis-orientation) for mobile robots.

The following arguments can be defined:

    'use_sim_time':
        Use simulation (Gazebo) clock if true
        (default: 'false')

    'urdf_file':
        Full path to the urdf file to use for launched nodes
        (default: '[/path/to/share]/ros_robomaster_description/urdf/robomaster.xacro')

    'namespace':
        Top-level namespace
        (default: '')

    'prefix':
        TF prefix
        (default: LaunchConfig('namespace') + '/')

    'use_joint_tester':
        Whether to launch joint_tester
        (default: 'false')

## Generate URDF from XACRO

The following comman will translate the xacro file to an urdf file.
```
rosrun xacro xacro robomaster.xacro > robomaster.urdf
```

## URDF to PDF
With the following command an pdf graphic can be generatef from the xacro file. This is helpful to visualise the TF-Tree.
```
urdf_to_graphiz robomaster.urdf 
```


## Fix display issue on an nvidia System 

On x86 Systems with an nvidia GPU an problem may occur. This can be fixed by reinstalling libglvnd-dev.
```
sudo apt install --reinstall libglvnd-dev
```




## Update Joint State 

Updating the joits can be done by publishing a ros message of the type "sensor_msgs/JointState". The name of the message must be in the source List of the Joint state publisher.S

```
rostopic pub /robomaster/wheel_joint_states sensor_msgs/JointState '{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: "interaction"}, name: ["Wheel_fr_joint"], position: [0.0], velocity: [0.0], effort: [0.0]}' --once

```