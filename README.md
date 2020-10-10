Trajectory planning for UR3 robotic arm
========================================
Author: Junqi Ren (Karl)   
Environment:Ubuntu 14.04 ROS Indigo  
  This package works on trajectory planning in Rviz and Gazebo for the 6 dof UR3 robotic arm. We can move the end effector to any specific point with the script "movetopoint.py" where the path from the initial point to the destination is a straight line. However, we could move the end effector of UR3 not only in lines but also in curves or splines with computing Cartesian path. Running "circle.py" could let the end effector move in a full circle and other splines or polynomials could also work in the same way.  
  The package works in Ubuntu 14.04 ROS Indigo. I also try to load the universal robot model and run the package in Ubuntu 16.04 ROS Kinetic but I met consecutive problems when I try to catkin make the original UR package and my own package. These problems are very common and a lof of engineers have also met these problems when compiling. I spent weeks but just only solved some of them. At last, I turned to Indigo and this time everything went smoothly. Thanks god.    
  I will elaborate how to solve the problems that I met in the following Troubleshooting part.  
## Dependencies:
  Go and check https://github.com/ros-industrial/universal_robot to get urdf,xacro,yaml and all the files you want. Please follow the instructions in http://wiki.ros.org/action/show/universal_robots?action=show&redirect=universal_robot and install all the dependencies. Wiki.ros.org has a comprehensive introduction of how is everything working in the UR package and you would find the solution to your problem after a careful reading(for the most of time).  
  However, no matter you are using Indigo or Kinetic,after you installing the universal robot package, you are very likely to encounter a problem when compiling,called *Action client not connected: PositionJointInterface_trajectory_controller/follow_joint_trajectory.*   
  Any edition in controller.yaml does not help. Actually you are lack of the gazebo-ros-control package. So just run `sudo apt-get install ros-$$$-gazebo-ros-pkgs ros-$$$-gazebo-ros-control` and $$$ is either indigo or kinetic.  
## Roll up our sleeves  
* initiate Gazebo in another terminal  
`source /opt/ros/indigo/setup.bash`  
`roslaunch ur_gazebo ur3.launch limited:=true`  
If you forget to setup the bash ,you may receive a warning like*No matching hardware interface found for 'hardware_interface/PositionJointInterface*  
* initiate moveit_planning   
`roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch sim:=true`    
* initiate Rviz  
`roslaunch ur3_moveit_config moveit_rviz.launch config:=true`  
* move to any specific point you want, like[x,y,z]=[0.4,0.2,0.4]  
`rosrun ur3_motionplanning movetopoint.py`
* move the end effector in a circle with a radius of 0.1  
`rosrun ur3_motionplanning circle.py`  
## Troubleshooting  
Like I said at the beginning, I use ROS Kinetic instead of Indigo at first and I could not even load the UR model in gazebo and rviz successfully. So basically, this troubleshooting part is for Kinetic users.  
*  *Check the version of your Gazebo.*    
gazebo-ros-control is a necessary package , otherwise, you will find the arm_controller node missing if you run rostopic list in your terminal. gazebo-ros-control is only dependent on Gazebo 7.X in Kinetics. I used Gazebo 8 at first and the warning *Action client not connected: PositionJointInterface_trajectory_controller/follow_joint_trajectory.*  kept showing up.  
*  *No p gain specified for pid.*    
Normally we don't have to set the pid parameters by ourselves. If you really want to set the pid parameters for all the six joints, you could build a gazebo_ros_control.yaml and set the pid parameters arbitrarily like p= 100.0, i=0.01, d= 10.0. Watch out, the vibration could be really intense and the joints could get misplaced.  
*  *The following packages have unmet dependencies*    
Try to use `sudo apt-get aptitude install --pkg name` and this command will help you install all the dependencies automatically.  
*   *Invalid Trajectory: start point deviates from current robot state more than...*  
When you receive any warning like this, it means the noise heavily affects your robotic arm. If you want to close the tolerance checking, you can either run `rosservice call /move_group/trajectory_execution/set_parameters "config: doubles: {name: 'allowed_start_tolerance', value: 0.0}" `mannually or add `rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.0)` to your moveit planning execution launch file. To be honest, you cannot generate a trajectory even you close the tolerance checking.  
## Future Plan
I will write controllers for more complex trajectories and learn more about API,OMPL and algorithms like RRT and RRT*. Moreover, ROS is a little bit unstable and I'm working on the controllers for UR arm in ROS2.




