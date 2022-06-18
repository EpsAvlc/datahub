cmake_minimum_required(VERSION 3.0.2)
project(datahub)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "RelWithDebInfo")
# -g 包含调试信息 
# -Wall 启动所有的 warning
# -O3 三级优化 最高的优化选项
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosunit
  rostest
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

# see here: https://github.com/PointCloudLibrary/pcl/issues/3680
# when this is fixed, we can remove the following 3 lines.
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include 
 LIBRARIES ${PROJECT_NAME}_lib
#  CATKIN_DEPENDS roscpp rospy
 DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  /usr/include/eigen3
)

######################## LIBS ####################
add_library(${PROJECT_NAME}_lib
  src/datahub.cc
  src/log_utils.cc
  src/read_write_mutex.cc
)
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

####################### Tests ####################
if(CATKIN_ENABLE_TESTING)
catkin_add_gtest(${PROJECT_NAME}_test
  tests/tests.cc
)
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  gtest_main
)
endif()

install(
  TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
