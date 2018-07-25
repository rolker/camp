cmake_minimum_required(VERSION 3.5.1)

project(AutonomousMissionPlanner)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 COMPONENTS Core Widgets Test)

if (Qt5Widgets_FOUND)
    if (Qt5Widgets_VERSION VERSION_LESS 5.6.0)
        message(FATAL_ERROR "Minimum supported Qt5 version is 5.6!")
    endif()
else()
    message(SEND_ERROR "The Qt5Widgets library could not be found!")
endif(Qt5Widgets_FOUND)

find_package(GDAL REQUIRED)
include_directories(${GDAL_INCLUDE_DIR})

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
endif(CMAKE_COMPILER_IS_GNUCXX)

option(AMP_USE_ROS "Build ROS components")

if(AMP_USE_ROS)
    find_package(catkin REQUIRED COMPONENTS roscpp asv_msgs)
    include_directories(${catkin_INCLUDE_DIRS})
    add_definitions(-DAMP_ROS)
endif()


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
    linestring.cpp
    polygon.cpp
    geographicsmissionitem.cpp
    modeltest.cpp
    surveyarea.cpp
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
    linestring.h
    polygon.h
    geographicsmissionitem.h
    surveyarea.h
)

if(AMP_USE_ROS)
    list(APPEND SOURCES roslink.cpp rosdetails.cpp)
    list(APPEND HEADERS roslink.h rosdetails.h)
endif()

set ( RESOURCES
 main.qrc
)

add_executable(AutonomousMissionPlanner ${HEADERS} ${SOURCES} ${RESOURCES})

qt5_use_modules(AutonomousMissionPlanner Widgets Positioning Svg Test)

target_link_libraries(AutonomousMissionPlanner ${QT_LIBRARIES} ${GDAL_LIBRARY} ${catkin_LIBRARIES})

INSTALL(TARGETS AutonomousMissionPlanner RUNTIME DESTINATION bin)
