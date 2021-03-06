cmake_minimum_required(VERSION 2.8.3)
project(dynamic_obstacle_avoidance_layers)

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  dynamic_reconfigure
  pluginlib
  roscpp
  geometry_msgs
  kalman_filter
)

add_message_files(
  FILES
  Boats.msg
  Boat.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/CustomLayerDynamic.cfg
  cfg/CustomLayerStatic.cfg
)

catkin_package(
    CATKIN_DEPENDS angles costmap_2d dynamic_reconfigure pluginlib roscpp kalman_filter
    INCLUDE_DIRS include
    LIBRARIES obstacle_layers
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_compile_options(-std=c++11)
## add cpp library
add_library(obstacle_layers
            src/social_layer.cpp
            src/custom_layer_dynamic.cpp
            src/custom_layer_static.cpp
)

add_executable(boats_from_lidar nodes/boats_from_lidar.cpp)

add_dependencies(
    obstacle_layers ${${PROJECT_NAME}_EXPORTED_TARGETS}
    boats_from_lidar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(boats_from_lidar ${catkin_LIBRARIES})

install(TARGETS
  boats_from_lidar
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

target_link_libraries(
    obstacle_layers ${catkin_LIBRARIES}
)

install(FILES costmap_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS obstacle_layers
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)