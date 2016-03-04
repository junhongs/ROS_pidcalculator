cmake_minimum_required(VERSION 2.8.3)
project(pid_controller)

## Find catkin and any catkin packages



find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs std_msgs genmsg message_generation cmake_modules
cv_bridge
image_transport
)
##find_package(catkin REQUIRED )
find_package(Eigen REQUIRED)


## Declare ROS messages and services
##add_message_files(FILES pos.msg)
##add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(
	DEPENDENCIES
	std_msgs
)

## Declare a catkin package
catkin_package(
	CATKIN_DEPENDS message_runtime DEPENDS Eigen
)


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(tested_value src/talker.cpp)
target_link_libraries(tested_value ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(tested_value pid_controller_cpp)

add_executable(reconstruction src/reconst_3d.cpp)
target_link_libraries(reconstruction ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(reconstruction pid_controller_cpp)


add_executable(shell src/rosshell.cpp src/param.cpp)
target_link_libraries(shell ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(shell pid_controller_cpp)

add_executable(pid_controller src/listener.cpp src/param.cpp src/calculation.cpp src/pidcontroller.cpp)
target_link_libraries(pid_controller ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(pid_controller pid_controller_cpp)
