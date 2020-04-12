This package is a further implementation of gazebo_ros_control
The new componenet of the package is AckermannRobotHWSim class
This class is instantiated as a plugin and depends on AckermannController
AckermannController respository and necessary documentation can be found on 22ARW github
Once the controller is set up correctly the plugin in the urdf file needs set to


<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/your_namespace_containing_parameters</robotNamespace>
    <robotSimType>gazebo_ros_control/AckermannRobotHWSim</robotSimType>
</plugin>



# Gazebo ros_control Interfaces

This is a ROS package for integrating the `ros_control` controller architecture
with the [Gazebo](http://gazebosim.org/) simulator.

This package provides a Gazebo plugin which instantiates a ros_control
controller manager and connects it to a Gazebo model.

[Documentation](http://gazebosim.org/tutorials?tut=ros_control) is provided on Gazebo's website.

## Future Direction

 - Implement transmissions

This package is a further implementation of gazebo_ros_control
The new componenet of the package is AckermannRobotHWSim class
This class is instantiated as a plugin and depends on AckermannController
AckermannController respository and necessary documentation can be found on 22ARW github
Once the controller is set up correctly the plugin in the urdf file needs set to


<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/your_namespace_containing_parameters</robotNamespace>
    <robotSimType>gazebo_ros_control/AckermannRobotHWSim</robotSimType>
</plugin>
