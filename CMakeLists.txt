cmake_minimum_required(VERSION 2.8.3)

project(robot_assisted_cleaning_solution)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
   sensor_msgs
   assisted_cleaning_solution
)

## catkin_package(CATKIN_DEPENDS message_runtime...)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
   scripts/range_pub.py
   scripts/odom_pub.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(FILES
  maps/office_map.yaml
  maps/office_map.pgm
  maps/one_lidar.pgm
  maps/one_lidar.yaml

  rviz/amcl.rviz
  rviz/gmapping.rviz
  rviz/range.rviz
  rviz/robot_model.rviz
  rviz/navigation.rviz
  rviz/scan.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)