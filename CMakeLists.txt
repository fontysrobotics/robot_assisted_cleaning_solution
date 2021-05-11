cmake_minimum_required(VERSION 2.8.3)

project(robot_assisted_cleaning_solution)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

## catkin_package(CATKIN_DEPENDS message_runtime...)

add_message_files(
  FILES
  Task.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(FILES
  maps/office_map.yaml
  maps/office_map.pgm
  maps/half_office.yaml
  maps/half_office.pgm

  rviz/amcl.rviz
  rviz/gmapping.rviz
  rviz/range.rviz
  rviz/robot_model.rviz
  rviz/navigation.rviz
  rviz/scan.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)