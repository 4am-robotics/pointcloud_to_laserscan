cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_to_laserscan)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  nodelet
  roscpp
  sensor_msgs
  tf
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_to_laserscan ipa_pointcloud_to_laserscan roi_outlier_removal
  CATKIN_DEPENDS roscpp message_filters nodelet sensor_msgs tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(frame_publisher src/frame_publisher.cpp)
target_link_libraries(frame_publisher ${catkin_LIBRARIES})

add_executable(frame_publisher_node src/frame_publisher_node.cpp)
target_link_libraries(frame_publisher_node frame_publisher ${catkin_LIBRARIES})

add_library(roi_outlier_removal src/roi_outlier_removal_nodelet.cpp)
target_link_libraries(roi_outlier_removal ${catkin_LIBRARIES})

add_library(pointcloud_to_laserscan src/pointcloud_to_laserscan_nodelet.cpp)
target_link_libraries(pointcloud_to_laserscan ${catkin_LIBRARIES})

add_executable(pointcloud_to_laserscan_node src/pointcloud_to_laserscan_node.cpp)
target_link_libraries(pointcloud_to_laserscan_node pointcloud_to_laserscan ${catkin_LIBRARIES})

add_library(ipa_pointcloud_to_laserscan src/ipa_pointcloud_to_laserscan_nodelet.cpp src/scan_outlier_removal_filter.cpp)
target_link_libraries(ipa_pointcloud_to_laserscan ${catkin_LIBRARIES})

add_executable(ipa_pointcloud_to_laserscan_node src/ipa_pointcloud_to_laserscan_node.cpp src/scan_outlier_removal_filter.cpp)
target_link_libraries(ipa_pointcloud_to_laserscan_node ipa_pointcloud_to_laserscan ${catkin_LIBRARIES})

install(TARGETS pointcloud_to_laserscan pointcloud_to_laserscan_node ipa_pointcloud_to_laserscan ipa_pointcloud_to_laserscan_node 
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
