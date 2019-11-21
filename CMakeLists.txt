cmake_minimum_required(VERSION 2.8.3)
project(pff_ped)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(astra_people src/astra_people.cpp)
target_link_libraries(astra_people ${catkin_LIBRARIES})

add_executable(velodyne_people src/velodyne_people.cpp)
target_link_libraries(velodyne_people ${catkin_LIBRARIES})
