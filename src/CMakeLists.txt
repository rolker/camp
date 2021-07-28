set(SOURCES
    main.cpp
    mainwindow.cpp
    autonomousvehicleproject.cpp
    backgroundraster.cpp
    georeferenced.cpp
    waypoint.cpp
    projectview.cpp
    trackline.cpp
    geographicsitem.cpp
    surveypattern.cpp
    detailsview.cpp
    waypointdetails.cpp
    tracklinedetails.cpp
    surveypatterndetails.cpp
    platform_manager/platform.cpp
    platform_manager/platform_manager.cpp
    missionitem.cpp
    backgrounddetails.cpp
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
    radardisplay.cpp
    ship_track.cpp
    ais/ais_contact.cpp
    ais/ais_manager.cpp
    helm_manager/helm_manager.cpp
    sound_play/sound_play_widget.cpp
    sound_play/speech_alerts.cpp
    roslink.cpp
    nav_source.cpp
    mission_manager/mission_manager.cpp
    geoviz/geoviz_display.cpp
)

set(HEADERS
    mainwindow.h
    autonomousvehicleproject.h
    backgroundraster.h
    georeferenced.h
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
    radardisplay.h
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
)

set ( RESOURCES
 main.qrc
)

add_executable(CCOMAutonomousMissionPlanner ${HEADERS} ${SOURCES} ${RESOURCES})

add_dependencies(CCOMAutonomousMissionPlanner ${catkin_EXPORTED_TARGETS})

qt5_use_modules(CCOMAutonomousMissionPlanner Widgets Positioning Svg Test)

target_link_libraries(CCOMAutonomousMissionPlanner ${QT_LIBRARIES} ${GDAL_LIBRARY} ${catkin_LIBRARIES})

INSTALL(TARGETS CCOMAutonomousMissionPlanner RUNTIME DESTINATION bin)


#rqt plugins

find_package(class_loader)

add_subdirectory(helm_manager)
