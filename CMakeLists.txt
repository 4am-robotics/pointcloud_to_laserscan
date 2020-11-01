cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_to_laserscan)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  nodelet
  pcl_ros
  roscpp
  sensor_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ipa_${PROJECT_NAME} roi_outlier_removal
  CATKIN_DEPENDS message_filters nodelet pcl_ros roscpp sensor_msgs tf tf2 tf2_geometry_msgs tf2_ros tf2_sensor_msgs
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

add_library(${PROJECT_NAME} src/${PROJECT_NAME}_nodelet.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(ipa_${PROJECT_NAME} src/ipa_${PROJECT_NAME}_nodelet.cpp src/scan_outlier_removal_filter.cpp)
target_link_libraries(ipa_${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(ipa_${PROJECT_NAME}_node src/ipa_${PROJECT_NAME}_node.cpp src/scan_outlier_removal_filter.cpp)
target_link_libraries(ipa_${PROJECT_NAME}_node ipa_${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS frame_publisher
                frame_publisher_node
                ipa_${PROJECT_NAME}
                ipa_${PROJECT_NAME}_node
                ${PROJECT_NAME}
                ${PROJECT_NAME}_node
                roi_outlier_removal
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
