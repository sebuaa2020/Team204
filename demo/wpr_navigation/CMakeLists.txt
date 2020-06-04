cmake_minimum_required(VERSION 2.8.3)
project(wpr_navigation)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	controller_manager
	gazebo_ros
	move_base
	map_server
	amcl
	gmapping
	dwa_local_planner
	wpr_msgs
)

catkin_package(
   CATKIN_DEPENDS urdf xacro
)



include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(navigation
  src/navigation.cpp
)
# add_dependencies(navigation wpr_msgs_generate_messages_cpp)
add_dependencies(navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation
  ${catkin_LIBRARIES}
)
