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
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(project11 REQUIRED)
find_package(project11_msgs REQUIRED)
find_package(project11_nav_msgs REQUIRED)
find_package(sound_play REQUIRED)
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

find_package(Qt5 COMPONENTS Core Widgets Test Concurrent Positioning Network Svg Xml)

if (Qt5Widgets_FOUND)
    if (Qt5Widgets_VERSION VERSION_LESS 5.6.0)
        message(FATAL_ERROR "Minimum supported Qt5 version is 5.6!")
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
    src/autonomousvehicleproject.cpp
    src/avoid_area.cpp
    src/backgrounddetails.cpp
    src/backgroundraster.cpp
    src/detailsview.cpp
    src/geographicsitem.cpp
    src/georeferenced.cpp
    src/grids/grid.cpp
    src/grids/grid_manager.cpp
    src/main.cpp
    src/mainwindow.cpp
    src/markers/markers.cpp
    src/markers/markers_manager.cpp
    src/missionitem.cpp
    src/mission_manager/mission_manager.cpp
    src/orbit.cpp
    src/orbitdetails.cpp
    src/platform_manager/platform.cpp
    src/platform_manager/platform_manager.cpp
    src/projectview.cpp
    src/ros/node_thread.cpp
    src/ros/ros_object.cpp
    src/ros/ros_widget.cpp
    src/searchpattern.cpp
    src/sound_play/sound_play_widget.cpp
    src/sound_play/speech_alerts.cpp
    src/surveypattern.cpp
    src/surveypatterndetails.cpp
    src/trackline.cpp
    src/tracklinedetails.cpp
    src/waypoint.cpp
    src/waypointdetails.cpp

    src/vector/vectordataset.cpp
    src/group.cpp
    src/vector/point.cpp
    src/vector/linestring.cpp
    src/vector/polygon.cpp
    src/geographicsmissionitem.cpp
    src/surveyarea.cpp
    src/measuringtool.cpp
    src/behavior.cpp
    src/behaviordetails.cpp
    src/astar.cpp
    src/ship_track.cpp
    src/ais/ais_contact.cpp
    src/ais/ais_manager.cpp
    src/helm_manager/helm_manager.cpp
    src/roslink.cpp
    src/nav_source.cpp
)

# src/radar/radar_display.cpp
# src/radar/radar_manager.cpp

set ( RESOURCES
    src/main.qrc
)

set (UIS
    src/behaviordetails.ui
    src/main/camp_main_window.ui
    src/sonar_manager/sonar_manager.ui
    src/sound_play/sound_play_widget.ui
    src/platform_manager/platform.ui
    src/platform_manager/platform_manager.ui
    src/orbitdetails.ui
    src/mainwindow.ui
    src/markers/markers.ui
    src/markers/markers_manager.ui
    src/waypointdetails.ui
    src/grids/grid.ui
    src/grids/grid_manager.ui
    src/helm_manager/helm_manager_config.ui
    src/helm_manager/helm_manager.ui
    src/backgrounddetails.ui
    src/roslink.ui
    src/surveypatterndetails.ui
    src/mission_manager/mission_manager.ui
    src/ais/ais_manager.ui
    src/tracklinedetails.ui
)

add_executable(CCOMAutonomousMissionPlanner ${SOURCES} ${RESOURCES} ${UIS})

target_include_directories(CCOMAutonomousMissionPlanner
    PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(CCOMAutonomousMissionPlanner
    grid_map_ros
    marine_ais_msgs
    nav_msgs
    project11
    project11_msgs
    project11_nav_msgs
    rclcpp
    sound_play
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

#find_package(class_loader)



set(rqt_helm_manager_SRCS
  src/helm_manager/rqt_helm_manager.cpp
  src/helm_manager/helm_manager.cpp
)

# set(rqt_helm_manager_HDRS
#   helm_manager.h
#   rqt_helm_manager.h
# )

# set(rqt_helm_manager_UIS
#     src/helm_manager/helm_manager.ui
#     src/helm_manager/rqt_helm_manager.ui
# )

add_library(rqt_helm_manager SHARED
  ${rqt_helm_manager_SRCS}
)

target_include_directories(rqt_helm_manager
    PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/helm_manager
)

ament_target_dependencies(rqt_helm_manager
  project11_msgs
  rqt_gui_cpp
  rclcpp
)

target_link_libraries(rqt_helm_manager
  ${QT_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
)

#class_loader_hide_library_symbols(rqt_helm_manager)

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


# test map view

set( CAMP_SOURCES
    src/background/background_manager.cpp
    src/main/cached_file_loader.cpp
    src/main/camp_main_window.cpp
    src/main/main.cpp
    src/map/map.cpp
    src/map/map_item.cpp
    src/map/map_item_mime_data.cpp
    src/map/layer_list.cpp
    src/map/layer.cpp
    src/map_tiles/cached_tile_loader.cpp
    src/map_tiles/map_tiles.cpp
    src/map_tiles/tile.cpp
    src/map_tiles/tile_address.cpp
    src/map_tiles/tile_layout.cpp
    src/map_tiles/osm.cpp
    src/map_tree_view/map_item_delegate.cpp
    src/map_tree_view/map_tree_view.cpp
    src/map_view/map_view.cpp
    src/map_view/web_mercator.cpp
    src/raster/raster_layer.cpp
    src/ros/grids/grid_manager.cpp
    src/ros/grids/grid_map.cpp
    src/ros/layer.cpp
    src/ros/markers/marker.cpp
    src/ros/markers/marker_namespace.cpp
    src/ros/markers/markers.cpp
    src/ros/markers/markers_manager.cpp
    src/ros/node_manager.cpp
    src/ros/node_thread.cpp
    src/tools/layer_manager.cpp
    src/tools/map_tool.cpp
    src/tools/tools_manager.cpp
    src/wmts/capabilities.cpp
    src/wmts/layer.cpp
    src/wmts/tile_matrix_set.cpp
)




add_executable(camp2 ${CAMP_SOURCES})

target_include_directories(camp2
    PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(camp2
    geometry_msgs
    grid_map_ros
    marine_ais_msgs
    project11
    project11_msgs
    project11_nav_msgs
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


ament_package(
)

