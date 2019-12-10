#-------------------------------------------------
#
# Project created by QtCreator 2019-01-12T12:27:11
#
#-------------------------------------------------

QT       += core gui

CONFIG += c++14

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TP3
TEMPLATE = app

SOURCES += main.cpp \
    glarea.cpp \
    princ.cpp \
    sphere.cpp \
    globject.cpp \
    box.cpp \

HEADERS  += \
    glarea.h \
    princ.h \
    globject.h \
    box.h \
    sphere.h

FORMS    += \
    princ.ui

RESOURCES += \
    ressources.qrc

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += bullet
