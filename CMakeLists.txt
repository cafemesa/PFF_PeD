cmake_minimum_required(VERSION 2.8.3)
project(pff_ped)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
  tf
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(astra_mov_people src/astra_mov_people.cpp)
target_link_libraries(astra_mov_people ${catkin_LIBRARIES})

add_executable(velodyne_mov_people src/velodyne_mov_people.cpp)
target_link_libraries(velodyne_mov_people ${catkin_LIBRARIES})

add_executable(velodyne_people src/velodyne_people.cpp)
target_link_libraries(velodyne_people ${catkin_LIBRARIES})

add_executable(read_txt src/read_txt.cpp)
target_link_libraries(read_txt ${catkin_LIBRARIES})

add_executable(velodyne_doors src/velodyne_doors.cpp)
target_link_libraries(velodyne_doors ${catkin_LIBRARIES})

add_executable(path src/path_sub.cpp)
target_link_libraries(path ${catkin_LIBRARIES})



add_executable(velodyne_chairs src/velodyne_chairs.cpp)
target_link_libraries(velodyne_chairs ${catkin_LIBRARIES})






