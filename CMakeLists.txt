cmake_minimum_required(VERSION 3.5.1)
project(camp)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    geographic_visualization_msgs
    grid_map_ros
    marine_ais_msgs
    marine_sensor_msgs
    project11_msgs
    project11_nav_msgs
    roscpp
    rosgraph_msgs
    rqt_gui
    rqt_gui_cpp
    sound_play
    tf2
    tf2_geometry_msgs
)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 COMPONENTS Core Widgets Test Concurrent Network Xml)

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

add_subdirectory(src)
