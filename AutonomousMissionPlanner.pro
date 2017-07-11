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
    surveypattern.cpp \
    detailsview.cpp \
    waypointdetails.cpp \
    tracklinedetails.cpp \
    surveypatterndetails.cpp \
    platform.cpp \
    missionitem.cpp \
    platformdetails.cpp \
    backgrounddetails.cpp

HEADERS  += mainwindow.h \
    autonomousvehicleproject.h \
    backgroundraster.h \
    georeferenced.h \
    waypoint.h \
    projectview.h \
    trackline.h \
    geographicsitem.h \
    surveypattern.h \
    detailsview.h \
    waypointdetails.h \
    tracklinedetails.h \
    surveypatterndetails.h \
    platform.h \
    missionitem.h \
    platformdetails.h \
    backgrounddetails.h

FORMS    += mainwindow.ui \
    waypointdetails.ui \
    tracklinedetails.ui \
    surveypatterndetails.ui \
    platformdetails.ui \
    backgrounddetails.ui

unix {
    INCLUDEPATH = /usr/include/gdal
}
darwin {
     INCLUDEPATH = /Library/Frameworks/GDAL.framework/Versions/2.1/Headers
     LIBS += -L /Library/Frameworks/GDAL.framework/Versions/2.1/unix/lib
     LIBS += -lgdal
}

LIBS += -lgdal
CONFIG += c++11
