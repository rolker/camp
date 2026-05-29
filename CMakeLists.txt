cmake_minimum_required(VERSION 3.5.1)

project(camp)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(marine_ais_msgs REQUIRED)
find_package(marine_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(marine_autonomy REQUIRED)
find_package(marine_interfaces REQUIRED)

#find_package(sound_play REQUIRED) pending port to ROS2
find_package(qt_gui_cpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 5.15 COMPONENTS Core Widgets Test Concurrent Positioning Network Svg Xml)

if (Qt5Widgets_FOUND)
    if (Qt5Widgets_VERSION VERSION_LESS 5.15.0)
        message(FATAL_ERROR "Minimum supported Qt5 version is 5.15 "
            "(camp_ros bridges use the Qt 5.10+ functor overload of "
            "QMetaObject::invokeMethod; ROS 2 Jazzy ships Qt 5.15).")
    endif()
else()
    message(SEND_ERROR "The Qt5Widgets library could not be found!")
endif(Qt5Widgets_FOUND)

find_package(GDAL REQUIRED)
include_directories(${GDAL_INCLUDE_DIR})

find_package(Doxygen)

if (DOXYGEN_FOUND)
    doxygen_add_docs(
        docs
        ${PROJECT_SOURCE_DIR}
    )
endif()

set(SOURCES
    src/camp/autonomousvehicleproject.cpp
    src/camp/avoid_area.cpp
    src/camp/backgrounddetails.cpp
    src/camp/backgroundraster.cpp
    src/camp/detailsview.cpp
    src/camp/geographicsitem.cpp
    src/camp/georeferenced.cpp
    src/camp/grids/grid.cpp
    src/camp/grids/grid_manager.cpp
    src/camp/main.cpp
    src/camp/mainwindow.cpp
    src/camp/markers/markers.cpp
    src/camp/markers/markers_manager.cpp
    src/camp/collision_monitor/collision_monitor.cpp
    src/camp/collision_monitor/collision_monitor_manager.cpp
    src/camp/missionitem.cpp
    src/camp/mission_manager/mission_manager.cpp
    src/camp/orbit.cpp
    src/camp/orbitdetails.cpp
    src/camp/platform_manager/platform.cpp
    src/camp/platform_manager/platform_manager.cpp
    src/camp/projectview.cpp
    src/camp/ros/node_thread.cpp
    src/camp/ros/ros_context.cpp
    src/camp/ros/ros_object.cpp
    src/camp/ros/ros_widget.cpp
    src/camp/searchpattern.cpp
    # src/camp/sound_play/sound_play_widget.cpp
    # src/camp/sound_play/speech_alerts.cpp
    src/camp/surveypattern.cpp
    src/camp/surveypatterndetails.cpp
    src/camp/trackline.cpp
    src/camp/tracklinedetails.cpp
    src/camp/waypoint.cpp
    src/camp/waypointdetails.cpp

    src/camp/vector/vectordataset.cpp
    src/camp/group.cpp
    src/camp/vector/point.cpp
    src/camp/vector/linestring.cpp
    src/camp/vector/polygon.cpp
    src/camp/geographicsmissionitem.cpp
    src/camp/surveyarea.cpp
    src/camp/measuringtool.cpp
    src/camp/behavior.cpp
    src/camp/behaviordetails.cpp
    src/camp/astar.cpp
    src/camp/ship_track.cpp
    src/camp/ais/ais_contact.cpp
    src/camp/ais/ais_manager.cpp
    src/camp/helm_manager/helm_manager.cpp
    src/camp/roslink.cpp
    src/camp/nav_source.cpp
)

# src/radar/radar_display.cpp
# src/radar/radar_manager.cpp

set ( RESOURCES
    src/camp/main.qrc
)

set (UIS
    src/camp/behaviordetails.ui
    src/camp/sonar_manager/sonar_manager.ui
    # src/camp/sound_play/sound_play_widget.ui
    src/camp/platform_manager/platform.ui
    src/camp/platform_manager/platform_manager.ui
    src/camp/orbitdetails.ui
    src/camp/mainwindow.ui
    src/camp/markers/markers.ui
    src/camp/markers/markers_manager.ui
    src/camp/collision_monitor/collision_monitor.ui
    src/camp/collision_monitor/collision_monitor_manager.ui
    src/camp/waypointdetails.ui
    src/camp/grids/grid.ui
    src/camp/grids/grid_manager.ui
    src/camp/helm_manager/helm_manager_config.ui
    src/camp/helm_manager/helm_manager.ui
    src/camp/backgrounddetails.ui
    src/camp/roslink.ui
    src/camp/surveypatterndetails.ui
    src/camp/mission_manager/mission_manager.ui
    src/camp/ais/ais_manager.ui
    src/camp/tracklinedetails.ui
)

add_executable(CCOMAutonomousMissionPlanner ${SOURCES} ${RESOURCES} ${UIS})

target_include_directories(CCOMAutonomousMissionPlanner
    PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp
)

ament_target_dependencies(CCOMAutonomousMissionPlanner
    geometry_msgs
    grid_map_ros
    message_filters
    marine_ais_msgs
    nav_msgs
    nav2_msgs
    marine_autonomy
    marine_interfaces

    rclcpp
    # sound_play
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

target_link_libraries(CCOMAutonomousMissionPlanner
    Qt5::Widgets
    Qt5::Concurrent
    Qt5::Network
    Qt5::Positioning
    Qt5::Svg
)

target_link_libraries(CCOMAutonomousMissionPlanner ${QT_LIBRARIES} ${GDAL_LIBRARY} yaml-cpp)

install (TARGETS CCOMAutonomousMissionPlanner
    DESTINATION lib/${PROJECT_NAME})


#rqt plugins

set(rqt_helm_manager_SRCS
  src/camp/helm_manager/rqt_helm_manager.cpp
  src/camp/helm_manager/helm_manager.cpp
)

add_library(rqt_helm_manager SHARED
  ${rqt_helm_manager_SRCS}
)

target_include_directories(rqt_helm_manager
    PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp/helm_manager
)

ament_target_dependencies(rqt_helm_manager
  marine_interfaces
  rqt_gui_cpp
  rclcpp
)

target_link_libraries(rqt_helm_manager
  ${QT_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
)

install(FILES
  plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS rqt_helm_manager
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


pluginlib_export_plugin_description_file(rqt_gui "plugins.xml")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


# camp2

set( CAMP2_SOURCES
    src/camp2/background/background_manager.cpp
    src/camp2/main/main_window.cpp
    src/camp2/main/main.cpp
    src/camp2/map/map.cpp
    src/camp2/map/map_item.cpp
    src/camp2/map/map_item_mime_data.cpp
    src/camp2/map/layer_list.cpp
    src/camp2/map/layer.cpp
    src/camp2/map_tiles/cached_tile_loader.cpp
    src/camp2/map_tiles/map_tiles.cpp
    src/camp2/map_tiles/tile.cpp
    src/camp2/map_tiles/tile_address.cpp
    src/camp2/map_tiles/tile_layout.cpp
    src/camp2/map_tiles/osm.cpp
    src/camp2/map_tree_view/map_item_delegate.cpp
    src/camp2/map_tree_view/map_tree_view.cpp
    src/camp2/map_view/map_view.cpp
    src/camp2/map_view/web_mercator.cpp
    src/camp2/raster/raster_layer.cpp
    src/camp2/ros/geometry/geometry_manager.cpp
    src/camp2/ros/geometry/polygon.cpp
    src/camp2/ros/graph_thread.cpp
    src/camp2/ros/grids/grid_layer.cpp
    src/camp2/ros/grids/grid_manager.cpp
    src/camp2/ros/grids/grid_map.cpp
    src/camp2/ros/grids/occupancy_grid.cpp
    src/camp2/ros/layer.cpp
    src/camp2/ros/markers/marker.cpp
    src/camp2/ros/markers/marker_namespace.cpp
    src/camp2/ros/markers/markers.cpp
    src/camp2/ros/markers/markers_manager.cpp
    src/camp2/ros/name.cpp
    src/camp2/ros/names_manager.cpp
    src/camp2/ros/node.cpp
    src/camp2/ros/node_thread.cpp
    src/camp2/tools/layer_manager.cpp
    src/camp2/tools/map_tool.cpp
    src/camp2/tools/tools_manager.cpp
    src/camp2/util/cached_file_loader.cpp
    src/camp2/wmts/capabilities.cpp
    src/camp2/wmts/layer.cpp
    src/camp2/wmts/tile_matrix_set.cpp
)

add_executable(camp2 ${CAMP2_SOURCES})

target_include_directories(camp2
    PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp2
)

ament_target_dependencies(camp2
    geometry_msgs
    grid_map_ros
    marine_ais_msgs
    marine_autonomy
    marine_interfaces

    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
)

target_link_libraries(camp2
    ${QT_LIBRARIES}
    ${GDAL_LIBRARY}
    Qt5::Core
    Qt5::Widgets
    Qt5::Concurrent
    Qt5::Network
    Qt5::Test
    Qt5::Xml
    Qt5::Positioning
)

install (TARGETS camp2
    DESTINATION lib/${PROJECT_NAME})


install (DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install (DIRECTORY workspace
    DESTINATION share/${PROJECT_NAME})


# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------
# Tests for the camp_ros bridge templates and the ports built on them.
# See docs/decisions/0001-topicbridge-and-executor-contract.md for the
# threading/TF contract these tests pin down.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  # TopicBridge / TfDispatcher / RosContext
  ament_add_gtest(test_topic_bridge
    test/test_topic_bridge.cpp
    src/camp/ros/ros_context.cpp
  )
  target_include_directories(test_topic_bridge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp
  )
  ament_target_dependencies(test_topic_bridge
    rclcpp
    tf2_ros
    tf2_geometry_msgs
    message_filters
    std_msgs
    geometry_msgs
    visualization_msgs
  )
  target_link_libraries(test_topic_bridge
    Qt5::Core
    Qt5::Test
  )

  # Marker converter unit tests
  ament_add_gtest(test_markers_converter
    test/test_markers_converter.cpp
  )
  target_include_directories(test_markers_converter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp
  )
  ament_target_dependencies(test_markers_converter
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    visualization_msgs
    marine_autonomy
  )
  target_link_libraries(test_markers_converter
    Qt5::Core
    Qt5::Positioning
  )

  # Collision-monitor polygon converter unit tests
  ament_add_gtest(test_collision_monitor_converter
    test/test_collision_monitor_converter.cpp
  )
  target_include_directories(test_collision_monitor_converter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp
  )
  ament_target_dependencies(test_collision_monitor_converter
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    marine_autonomy
  )
  target_link_libraries(test_collision_monitor_converter
    Qt5::Core
    Qt5::Positioning
  )
endif()


ament_package(
)

