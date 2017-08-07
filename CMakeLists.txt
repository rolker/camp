cmake_minimum_required(VERSION 3.7.2)

project(AutonomousMissionPlanner)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 COMPONENTS Core Widgets)

find_package(GDAL REQUIRED)
include_directories(${GDAL_INCLUDE_DIR})

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
endif(CMAKE_COMPILER_IS_GNUCXX)


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
    vectordataset.cpp
    group.cpp
    point.cpp
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
    vectordataset.h
    group.h
    point.h
)

set ( RESOURCES
 main.qrc
)

add_executable(AutonomousMissionPlanner ${HEADERS} ${SOURCES} ${RESOURCES})

qt5_use_modules(AutonomousMissionPlanner Widgets Positioning Svg)

target_link_libraries(AutonomousMissionPlanner ${QT_LIBRARIES} ${GDAL_LIBRARY})

INSTALL(TARGETS AutonomousMissionPlanner RUNTIME DESTINATION bin)
