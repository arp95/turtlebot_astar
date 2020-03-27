cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_astar)

add_compile_options(-std=c++11)

set(PACKAGE_DEPENDENCIES
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  rostest
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

include_directories( 
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

set(LINK_LIBS
    ${catkin_LIBRARIES}
)


#add_executable(turtlebot_astar src/main.cpp)
#target_link_libraries(turtlebot_astar ${catkin_LIBRARIES})

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
#  add_rostest_gtest(turtlebot_navigator_test test/test.launch test/main.cpp test/NavigationTest.cpp test/ObstacleDetectorTest.cpp src/Navigation.cpp src/ObstacleDetector.cpp)
#  target_link_libraries(turtlebot_navigator_test ${catkin_LIBRARIES})
#  add_dependencies(turtlebot_navigator turtlebot_navigator_test ${catkin_EXPORTED_TARGETS})
#endif()
