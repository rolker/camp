#-------------------------------------------------
#
# Project created by QtCreator 2016-10-14T13:13:12
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = AutonomousMissionPlanner
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    missioncanvas.cpp \
    autonomousvehicleproject.cpp \
    backgroundraster.cpp

HEADERS  += mainwindow.h \
    missioncanvas.h \
    autonomousvehicleproject.h \
    backgroundraster.h

FORMS    += mainwindow.ui

INCLUDEPATH = /usr/include/gdal
LIBS += -lgdal
CONFIG += c++11
