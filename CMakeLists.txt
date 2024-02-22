cmake_minimum_required(VERSION 3.5.1)

project(camp)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(marine_ais_msgs REQUIRED)
find_package(marine_sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(project11_msgs REQUIRED)
find_package(project11_nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 COMPONENTS Core Widgets Test Concurrent Positioning Network Xml)

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
    autonomousvehicleproject.cpp
    avoid_area.cpp
    backgrounddetails.cpp
    backgroundraster.cpp
    detailsview.cpp
    geographicsitem.cpp
    georeferenced.cpp
    grids/grid.cpp
    grids/grid_manager.cpp
    main.cpp
    mainwindow.cpp
    markers/markers.cpp
    markers/markers_manager.cpp
    missionitem.cpp
    mission_manager/mission_manager.cpp
    orbit.cpp
    orbitdetails.cpp
    platform_manager/platform.cpp
    platform_manager/platform_manager.cpp
    projectview.cpp
    radar/radar_display.cpp
    radar/radar_manager.cpp
    searchpattern.cpp
    surveypattern.cpp
    surveypatterndetails.cpp
    trackline.cpp
    tracklinedetails.cpp
    waypoint.cpp
    waypointdetails.cpp

    vector/vectordataset.cpp
    group.cpp
    vector/point.cpp
    vector/linestring.cpp
    vector/polygon.cpp
    geographicsmissionitem.cpp
    surveyarea.cpp
    measuringtool.cpp
    behavior.cpp
    behaviordetails.cpp
    astar.cpp
    ship_track.cpp
    ais/ais_contact.cpp
    ais/ais_manager.cpp
    helm_manager/helm_manager.cpp
    sound_play/sound_play_widget.cpp
    sound_play/speech_alerts.cpp
    roslink.cpp
    nav_source.cpp
)

set(HEADERS
    autonomousvehicleproject.h
    backgroundraster.h
    georeferenced.h
    mainwindow.h
    grids/grid.h
    grids/grid_manager.h
    markers/markers.h
    markers/markers_manager.h
    orbit.h
    orbitdetails.h
    radar/radar_display.h
    radar/radar_manager.h
    waypoint.h
    projectview.h
    trackline.h
    geographicsitem.h
    surveypattern.h
    detailsview.h
    waypointdetails.h
    tracklinedetails.h
    surveypatterndetails.h
    platform_manager/platform.h
    platform_manager/platform_manager.h
    missionitem.h
    backgrounddetails.h
    vector/vectordataset.h
    group.h
    vector/point.h
    vector/linestring.h
    vector/polygon.h
    geographicsmissionitem.h
    surveyarea.h
    measuringtool.h
    behavior.h
    behaviordetails.h
    astar.h
    ship_track.h
    ais/ais_contact.h
    ais/ais_manager.h
    helm_manager/helm_manager.h
    sound_play/sound_play_widget.h
    sound_play/speech_alerts.h
    roslink.h
    nav_source.h
    mission_manager/mission_manager.h
    
    searchpattern.h
)

set ( RESOURCES
    main.qrc
)

# add_executable(CCOMAutonomousMissionPlanner ${HEADERS} ${SOURCES} ${RESOURCES})

# target_include_directories(CCOMAutonomousMissionPlanner PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
# )

# ament_target_dependencies(CCOMAutonomousMissionPlanner
#     marine_ais_msgs
#     project11_msgs
#     project11_nav_msgs
#     tf2
#     rclcpp
# )

# target_link_libraries(CCOMAutonomousMissionPlanner
#     Qt5::Core 
#     Qt5::Widgets
#     Qt5::Concurrent
#     Qt5::Network
#     Qt5::Positioning
# )

# target_link_libraries(CCOMAutonomousMissionPlanner ${QT_LIBRARIES} ${GDAL_LIBRARY} yaml-cpp)

# install (TARGETS CCOMAutonomousMissionPlanner
#     DESTINATION lib/${PROJECT_NAME})


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
    background/background_manager.cpp
    main/cached_file_loader.cpp
    main/camp_main_window.cpp
    main/main.cpp
    map/map.cpp
    map/map_item.cpp
    map/map_item_mime_data.cpp
    map/layer_list.cpp
    map/layer.cpp
    map_tiles/cached_tile_loader.cpp
    map_tiles/map_tiles.cpp
    map_tiles/tile.cpp
    map_tiles/tile_address.cpp
    map_tiles/tile_layout.cpp
    map_tiles/osm.cpp
    map_tree_view/map_item_delegate.cpp
    map_tree_view/map_tree_view.cpp
    map_view/map_view.cpp
    map_view/web_mercator.cpp
    raster/raster_layer.cpp
    ros/layer.cpp
    ros/node.cpp
    ros/node_manager.cpp
    ros/grids/grid_manager.cpp
    ros/grids/grid_map.cpp
    ros/markers/marker.cpp
    ros/markers/marker_namespace.cpp
    ros/markers/markers.cpp
    ros/markers/markers_manager.cpp
    tools/layer_manager.cpp
    tools/map_tool.cpp
    tools/tools_manager.cpp
    wmts/capabilities.cpp
    wmts/layer.cpp
    wmts/tile_matrix_set.cpp
)


# add_executable(camp2 ${CAMP_SOURCES})

# ament_target_dependencies(camp2
#     grid_map_ros
#     marine_ais_msgs
#     project11_msgs
#     project11_nav_msgs
#     rclcpp
#     tf2_ros
# )

# target_link_libraries(camp2
#     ${QT_LIBRARIES}
#     ${GDAL_LIBRARY}
#     Qt5::Core
#     Qt5::Widgets
#     Qt5::Concurrent
#     Qt5::Network
#     Qt5::Test
#     Qt5::Xml
#     Qt5::Positioning
# )

# install (TARGETS camp2
#     DESTINATION lib/${PROJECT_NAME})


ament_package(
)

