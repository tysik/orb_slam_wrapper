cmake_minimum_required(VERSION 2.8.3)
project(orb_slam_wrp)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs tf)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

catkin_package(
   INCLUDE_DIRS
   LIBRARIES 
   CATKIN_DEPENDS roscpp geometry_msgs tf
   DEPENDS
)

set(ORB_SLAM_PATH $ENV{HOME}/ORB_SLAM2)

set(LIBS
  roscpp
  rosconsole
  roscpp_serialization
  ${ORB_SLAM_PATH}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM_PATH}/Thirdparty/g2o/lib/libg2o.so
  ${ORB_SLAM_PATH}/lib/libORB_SLAM2.so
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
)

include_directories(
  include
  ${ORB_SLAM_PATH}/include
  ${ORB_SLAM_PATH}/ThirdParty
  ${ORB_SLAM_PATH}
  ${catkin_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

link_directories(
  ${ORB_SLAM_PATH}/lib
  ${catkin_LIBRARY_DIRS}
)

add_library(orb_slam src/orb_slam.cpp include/orb_slam.h)
target_link_libraries(orb_slam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ueye_api ${LIBS})
add_dependencies(orb_slam ${catkin_EXPORTED_TARGETS})

add_executable(orb_slam_node src/orb_slam_node.cpp)
target_link_libraries(orb_slam_node orb_slam)
