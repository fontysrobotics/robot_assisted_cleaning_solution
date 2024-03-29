# robot_assisted_cleaning_solution
This is a RoS Noetic package for a the ACS robot. 

This package contains: 
* Configuration data of joints
* 3D model URDF 
* Launch files. For testing, Mapping and Navigation
* Configurations files for the global planner: move_base 
* Configurations files for the local planner: teb_local_planner.

## Dependencies
	gmapping
	teleop_twist_keyboard
	map_server
	move_base
	teb_local_planner
	amcl

## Other packages used
### ira_laser_tools-master, Needed.
To use this package you need to install the ira_laser_tools-master package from source.
The file laserscan_multi_merger.launch should look like:

	<launch>
		<arg name="node_start_delay" default="5.0" />  
		<node pkg="ira_laser_tools" name="laserscan_multi_merger" type="laserscan_multi_merger" output="screen" launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@' ">
				<param name="destination_frame" value="base_footprint"/>
				<param name="cloud_destination_topic" value="/merged_cloud"/>
				<param name="scan_destination_topic" value="/scan"/>
				<param name="laserscan_topics" value ="/scan1 /scan2" /> <!-- LIST OF THE LASER SCAN TOPICS TO SUBSCRIBE -->
				<param name="angle_min" value="-3.14"/>
				<param name="angle_max" value="3.14"/>
				<param name="angle_increment" value="0.0064"/>
				<param name="scan_time" value="0.0333333"/>
				<param name="range_min" value="0.1"/>
				<param name="range_max" value="9.0"/>
		</node>
	</launch>
	
### office_test_environment, Optional.

This is a RoS noetic package for a test office environment. 
To use this package you need to install the office_test_environment package from source.
More information about the package in the git repo : https://github.com/fontysrobotics/office_test_environment.git
 
This package contains:
* Configuration data of joints
* 3D model URDF 
* Simple test launch file.

## Launch in gazebo
The robot will be spawned in the office test environment. When needed with the map of the enviorment

To start empty world gazebo with the robot:

	roslaunch robot_assisted_cleaning_solution gazebo.launch
    
To start the mapping with the robot:

	roslaunch robot_assisted_cleaning_solution mapping.launch
	rosrun teleop_twist_keyboard teleop_twist_keyboard.py

	rosrun map_server map_saver -f map
		
To start navigation with the robot:

	roslaunch robot_assisted_cleaning_solution navigation.launch
	
## Launch on real robot
In the launch files of mapping and navigation. In these launch files the launch of robot should be changed to real_robot.
From this:

	<include file="$(find robot_assisted_cleaning_solution)/launch/robot.launch"/>
	
To this:

	<include file="$(find robot_assisted_cleaning_solution)/launch/real_robot.launch"/>
	
In the real robot file the odom and sensors topic is being published in the correct format. The data from the sensors is published over the sensor topic by the microcontroller in the assisted cleaning solution package. The lidar data is being published by the rplidar package.
