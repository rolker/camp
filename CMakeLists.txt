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

find_package(Qt5 5.15 COMPONENTS Core Gui Widgets Test Concurrent Positioning Network Svg Xml)

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
    src/camp/depth_raster.cpp
    src/camp/detailsview.cpp
    src/camp/geographicsitem.cpp
    src/camp/georeferenced.cpp
    src/camp/main.cpp
    src/camp/mainwindow.cpp
    src/camp/collision_monitor/collision_monitor.cpp
    src/camp/collision_monitor/collision_monitor_manager.cpp
    src/camp/footprint/footprint.cpp
    src/camp/footprint/footprint_manager.cpp
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

set ( RESOURCES
    src/camp/main.qrc
)

set (UIS
    src/camp/behaviordetails.ui
    src/camp/platform_manager/platform.ui
    src/camp/platform_manager/platform_manager.ui
    src/camp/orbitdetails.ui
    src/camp/mainwindow.ui
    src/camp/collision_monitor/collision_monitor.ui
    src/camp/waypointdetails.ui
    src/camp/helm_manager/helm_manager_config.ui
    src/camp/helm_manager/helm_manager.ui
    src/camp/roslink.ui
    src/camp/surveypatterndetails.ui
    src/camp/mission_manager/mission_manager.ui
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

# [#59 PR3a] Deployed camp adopts the shared map framework. camp_map_ros links
# camp_map transitively and propagates the src/camp2 PUBLIC include path, so the
# Web-Mercator scene (camp::map::Map), web_mercator helpers, and raster::RasterLayer
# resolve from headers rooted at src/camp2 (e.g. "map/map.h"). See ADR-0002.
target_link_libraries(CCOMAutonomousMissionPlanner camp_map_ros)

install (TARGETS CCOMAutonomousMissionPlanner
    DESTINATION lib/${PROJECT_NAME})


#rqt plugins

set(rqt_helm_manager_SRCS
  src/camp/helm_manager/rqt_helm_manager.cpp
  src/camp/helm_manager/helm_manager.cpp
  src/camp/ros/ros_context.cpp
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
  tf2_ros
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


# ============================================================================
# libcamp_map — shared map substrate, pure Qt/GDAL (ROS-free)   [#59 PR2]
# ============================================================================
# camp2's self-contained map core: layer model, Web-Mercator scene, OSM/WMTS
# tiles, reprojected rasters, tree view, interaction tools. The ROS node is
# attached by the application layer (MainWindow) via Map::toolsManager(), so
# this library has no ROS dependency. Linked by the camp2 executable below and,
# from PR3a, by CCOMAutonomousMissionPlanner.

set(CAMP_MAP_SOURCES
    src/camp2/background/background_manager.cpp
    src/camp2/map/map.cpp
    src/camp2/map/map_item.cpp
    src/camp2/map/map_item_mime_data.cpp
    src/camp2/map/layer_list.cpp
    src/camp2/map/layer.cpp
    src/camp2/map/color_map.cpp
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
    src/camp2/raster/gggs_tile.cpp
    src/camp2/raster/gggs_tile_layer.cpp
    src/camp2/raster/gggs_store_layer.cpp
    src/camp2/tools/layer_manager.cpp
    src/camp2/tools/map_tool.cpp
    src/camp2/tools/tools_manager.cpp
    src/camp2/util/cached_file_loader.cpp
    src/camp2/wmts/capabilities.cpp
    src/camp2/wmts/layer.cpp
    src/camp2/wmts/tile_matrix_set.cpp
)

add_library(camp_map SHARED ${CAMP_MAP_SOURCES})

# src/camp2 on the include path is PUBLIC so it propagates to camp_map_ros and
# the executables (header includes are rooted at src/camp2, e.g. "map/map.h").
target_include_directories(camp_map PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp2
)

target_link_libraries(camp_map
    PUBLIC
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::Concurrent
        Qt5::Network
        Qt5::Xml
        Qt5::Positioning
    PRIVATE
        ${GDAL_LIBRARY}
)

install(TARGETS camp_map
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

# ============================================================================
# libcamp_map_ros — ROS-dependent map layers; links libcamp_map   [#59 PR2]
# ============================================================================
# The camp2 ROS layer set (geometry, grids, markers) + node/name plumbing.
# Boundary verified one-directional after inverting Map's node creation: the
# src/camp2 core has zero ROS includes; only this layer depends on
# rclcpp/tf2/messages.

set(CAMP_MAP_ROS_SOURCES
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
    src/camp2/ros/transform_cache.cpp
)

add_library(camp_map_ros SHARED ${CAMP_MAP_ROS_SOURCES})

# Plain signature (no PUBLIC keyword): ament_target_dependencies below uses the
# plain target_link_libraries form, and CMake forbids mixing the two on one
# target. The plain form still links camp_map transitively, so camp_map's
# PUBLIC Qt + include-dir usage requirements propagate downstream.
target_link_libraries(camp_map_ros camp_map)

ament_target_dependencies(camp_map_ros
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

install(TARGETS camp_map_ros
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

# ============================================================================
# camp2 executable — app shell only; links the shared libraries   [#59 PR2]
# ============================================================================

add_executable(camp2
    src/camp2/main/main.cpp
    src/camp2/main/main_window.cpp
)

target_include_directories(camp2 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp2
)

# Qt5::Test supplies QAbstractItemModelTester, used by main_window.cpp.
target_link_libraries(camp2 camp_map_ros Qt5::Test)

# camp_map_ros transitively brings camp_map + Qt. The ROS package deps are
# repeated here so main.cpp/main_window.cpp (which include ros/node.h directly)
# always resolve ROS headers, independent of ament's interface-propagation.
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


  # ColorMap facility unit tests (camp#63)
  ament_add_gtest(test_color_map
    test/test_color_map.cpp
  )
  target_link_libraries(test_color_map
    camp_map
    Qt5::Test
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

  # earth<-frame transform cache / staleness decision (#59 — TF-gap flicker fix)
  ament_add_gtest(test_transform_cache
    test/test_transform_cache.cpp
    src/camp2/ros/transform_cache.cpp
  )
  target_include_directories(test_transform_cache PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp2
  )
  ament_target_dependencies(test_transform_cache
    rclcpp
    tf2
    tf2_ros
    geometry_msgs
  )

  # DepthRaster safe-degradation (#59 PR6 — depth extracted from BackgroundRaster)
  ament_add_gtest(test_depth_raster
    test/test_depth_raster.cpp
    src/camp/depth_raster.cpp
    src/camp/georeferenced.cpp
  )
  target_include_directories(test_depth_raster PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp
  )
  target_link_libraries(test_depth_raster
    Qt5::Core
    Qt5::Positioning
    ${GDAL_LIBRARY}
  )

  # A* shoal avoidance + unknown-depth-as-obstacle (#59 PR6 — depth/A* decoupled
  # from BackgroundRaster). Pure std C++; no Qt/ROS link needed.
  ament_add_gtest(test_astar
    test/test_astar.cpp
    src/camp/astar.cpp
  )
  target_include_directories(test_astar PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp
  )

  # camp::map::Map item-model correctness (#59 — model-mutation paths added by
  # the BackgroundRaster retirement: stacked-layer inserts + the Layers-tab
  # Remove action). Lifts the camp2 sandbox's QAbstractItemModelTester check
  # into CI by linking the shared camp_map library.
  ament_add_gtest(test_map_model
    test/test_map_model.cpp
  )
  target_include_directories(test_map_model PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp2
  )
  target_link_libraries(test_map_model
    camp_map
    Qt5::Widgets
    Qt5::Test
  )

  # MapTiles auto-refresh timer + bounded-memory eviction (#99 Phase 2). The
  # bounded-memory case is the #98 tile-lifecycle regression gate: refresh cycles
  # must delete old Tile children, not accumulate them. Links the shared camp_map
  # library where MapTiles/Tile live.
  ament_add_gtest(test_map_tiles_refresh
    test/test_map_tiles_refresh.cpp
  )
  target_include_directories(test_map_tiles_refresh PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp2
  )
  target_link_libraries(test_map_tiles_refresh
    camp_map
    Qt5::Widgets
    Qt5::Test
  )

  # GGGS GPU-warp tile layer (camp#90 / I4): GggsTile extent-from-geotransform +
  # NoData/range handling, and the shader-warp == web_mercator::geoToMap parity
  # anchor. Links camp_map (GggsTile + web_mercator) and GDAL (synthetic tiles).
  ament_add_gtest(test_gggs_tile
    test/test_gggs_tile.cpp
  )
  target_include_directories(test_gggs_tile PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp2
  )
  target_link_libraries(test_gggs_tile
    camp_map
    Qt5::Core
    Qt5::Gui
    Qt5::Positioning
    ${GDAL_LIBRARY}
  )

  # GGGS offscreen-FBO render (camp#90 / I4): warps a synthetic tile in the
  # layer's own GL context and checks the QImage is non-empty + oriented.
  # Self-skips when no offscreen GL context is available.
  ament_add_gtest(test_gggs_render
    test/test_gggs_render.cpp
  )
  target_include_directories(test_gggs_render PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp2
  )
  target_link_libraries(test_gggs_render
    camp_map
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::Positioning
    ${GDAL_LIBRARY}
  )

  # [camp#102] GGGS tile-set default-off visibility + persistence round-trip
  # (non-GL state machine — the highest-risk logic of the lazy/async change).
  ament_add_gtest(test_gggs_visibility
    test/test_gggs_visibility.cpp
  )
  target_include_directories(test_gggs_visibility PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camp2
  )
  target_link_libraries(test_gggs_visibility
    camp_map
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::Positioning
    ${GDAL_LIBRARY}
  )
endif()


ament_package(
)

