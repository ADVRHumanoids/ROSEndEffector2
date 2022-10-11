# ROSEndEffector2
The [ROS End-Effector](https://github.com/ADVRHumanoids/ROSEndEffector) package but for ROS2  
Please refer to the ROS End-Effector documentation for ROS1 at https://advrhumanoids.github.io/ROSEndEffectorDocs/

Note: The rosee_msg repo is unique for ROS1 and ROS2 versions of ROSEE: simply change the branch of into `ros2` [link](https://github.com/ADVRHumanoids/rosee_msg/tree/ros2)


### Usage notes

To build: `colcon build`
To run: `ros2 launch end_effector find_actions_launch.xml hand_name:=test_ee`
