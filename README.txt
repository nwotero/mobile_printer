This is the repository for the Mobile 3D Printer project. The code utilizes the Robot Operating System and the Gazebo simulator. It is organized into the following ROS packages:

printer_control - This package contains the control schemes which map desired workspace trajectories into actuated joint trajectories.

printer_description - This package contains the URDF and SDF files which describe the spacial parameters and internal kinematics of the robot.

printer_gazebo - This package contains the bindings for this ROS project and the Gazebo simulator. Specifically, this code in this package interprets low level robot commands and actuates them in Gazebo.

printer_planner - This package contains the code which maps objects into desired trajectories. This includes object slicing, global planning, and local planning.

To contribute to this project, you will need to install ROS Hydro on your computer. Follow the instructions found here: http://wiki.ros.org/hydro/Installation