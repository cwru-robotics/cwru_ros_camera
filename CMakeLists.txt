cmake_minimum_required(VERSION 2.8.3)
project(cwru_ros_camera)

find_package(Gazebo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	gazebo_ros
	gazebo_plugins
	cv_bridge
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

link_directories(${GAZEBO_LIBRARY_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
	#LIBRARIES ${PROJECT_NAME}
	DEPENDS gazebo_ros
	CATKIN_DEPENDS gazebo_ros gazebo_plugins
)

add_library(cwru_ros_camera src/cwru_ros_camera.cpp)
target_link_libraries(cwru_ros_camera ${OpenCV_LIBS} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} CameraPlugin)
