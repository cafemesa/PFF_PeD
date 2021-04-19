cmake_minimum_required(VERSION 2.8.3)
project(pff_ped)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  laser_geometry
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(astra_ped src/astra.cpp)
target_link_libraries(astra_ped ${catkin_LIBRARIES})

add_executable(velodyne_ped src/velodyne.cpp)
target_link_libraries(velodyne_ped ${catkin_LIBRARIES})