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
    platform.cpp
    missionitem.cpp
    platformdetails.cpp
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
    rosdetails.cpp
    astar.cpp
    radardisplay.cpp
    ship_track.cpp
    ais/ais_contact.cpp
    ais/ais_manager.cpp
    helm_manager/helm_manager.cpp
    sound_play/sound_play_widget.cpp
    sound_play/speech_alerts.cpp
    roslink.cpp
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
    platform.h
    missionitem.h
    platformdetails.h
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
    rosdetails.h
    astar.h
    radardisplay.h
    ship_track.h
    ais/ais_contact.h
    ais/ais_manager.h
    helm_manager/helm_manager.h
    sound_play/sound_play_widget.h
    sound_play/speech_alerts.h
    roslink.h
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
