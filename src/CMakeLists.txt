set(SOURCES
    autonomousvehicleproject.cpp
    backgrounddetails.cpp
    backgroundraster.cpp
    detailsview.cpp
    geographicsitem.cpp
    georeferenced.cpp
    geoviz/geoviz_display.cpp
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
    modeltest.cpp
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
    geoviz/geoviz_display.h
    searchpattern.h
)

set ( RESOURCES
    main.qrc
)

add_executable(CCOMAutonomousMissionPlanner ${HEADERS} ${SOURCES} ${RESOURCES})

add_dependencies(CCOMAutonomousMissionPlanner ${catkin_EXPORTED_TARGETS})

qt5_use_modules(CCOMAutonomousMissionPlanner Widgets Positioning Svg Test Concurrent Network)

target_link_libraries(CCOMAutonomousMissionPlanner ${QT_LIBRARIES} ${GDAL_LIBRARY} ${catkin_LIBRARIES} yaml-cpp)

INSTALL(TARGETS CCOMAutonomousMissionPlanner RUNTIME DESTINATION bin)


#rqt plugins

find_package(class_loader)

add_subdirectory(helm_manager)


# test map view

set( TEST_MAP_VIEW_SOURCES
    map_tiles/cached_tile_loader.cpp
    map_tiles/map_tiles.cpp
    map_tiles/tile.cpp
    map_tiles/tile_address.cpp
    map_view/map_view.cpp
    map_view/web_mercator.cpp
    test/test_map_view.cpp
)


add_executable(test_map_view ${TEST_MAP_VIEW_SOURCES})
qt5_use_modules(test_map_view Widgets Positioning Concurrent Network)
target_link_libraries(test_map_view ${QT_LIBRARIES})
