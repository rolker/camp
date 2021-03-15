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
)

if(AMP_USE_ROS)
    list(APPEND SOURCES roslink.cpp )
    list(APPEND HEADERS roslink.h)
endif()

set ( RESOURCES
 main.qrc
)

add_executable(CCOMAutonomousMissionPlanner ${HEADERS} ${SOURCES} ${RESOURCES})

if(AMP_USE_ROS)
    add_dependencies(CCOMAutonomousMissionPlanner ${catkin_EXPORTED_TARGETS})
endif()

qt5_use_modules(CCOMAutonomousMissionPlanner Widgets Positioning Svg Test)

target_link_libraries(CCOMAutonomousMissionPlanner ${QT_LIBRARIES} ${GDAL_LIBRARY} ${catkin_LIBRARIES})

INSTALL(TARGETS CCOMAutonomousMissionPlanner RUNTIME DESTINATION bin)
