cmake_minimum_required(VERSION 2.8.3)
project(cam_sync)

add_definitions(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  dynamic_reconfigure
  flea3 REQUIRED        # needed for the test module
)

generate_dynamic_reconfigure_options(
  cfg/CamSyncDynConfig.cfg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )


add_library(${PROJECT_NAME} src/cam_sync.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(${PROJECT_NAME}_node src/cam_sync_node.cpp src/cam_sync.cpp)

# make sure configure headers are built before any node using them
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )


