cmake_minimum_required(VERSION 3.0.2)
project(euclidean_cluster)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  tf2_ros
  tf2_eigen  # 여기에 tf2_eigen 포함
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(euclidean_cluster_node src/euclidean_cluster.cpp)

target_link_libraries(euclidean_cluster_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(euclidean_cluster_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lidar_angle_robust src/lidar_angle_robust.cpp)

target_link_libraries(lidar_angle_robust
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(lidar_angle_robust ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(euclidean_cluster_box src/euclidean_cluster_box.cpp)

target_link_libraries(euclidean_cluster_box
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(euclidean_cluster_box ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(obstacle_detection src/obstacle_detection.cpp)

target_link_libraries(obstacle_detection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(obstacle_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(fixed_angle_lidar src/fixed_angle_lidar.cpp)

target_link_libraries(fixed_angle_lidar
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(fixed_angle_lidar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

