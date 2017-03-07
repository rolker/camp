#-------------------------------------------------
#
# Project created by QtCreator 2016-10-14T13:13:12
#
#-------------------------------------------------

QT       += core gui positioning

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = AutonomousMissionPlanner
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    autonomousvehicleproject.cpp \
    backgroundraster.cpp \
    georeferenced.cpp \
    waypoint.cpp \
    projectview.cpp \
    trackline.cpp \
    geographicsitem.cpp \
    surveypattern.cpp

HEADERS  += mainwindow.h \
    autonomousvehicleproject.h \
    backgroundraster.h \
    georeferenced.h \
    waypoint.h \
    projectview.h \
    trackline.h \
    geographicsitem.h \
    surveypattern.h

FORMS    += mainwindow.ui

unix {
    INCLUDEPATH = /usr/include/gdal
}

LIBS += -lgdal
CONFIG += c++11
