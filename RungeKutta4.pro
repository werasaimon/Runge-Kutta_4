TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

QT += core gui opengl


LIBS += -lopengl32 -lglu32 -lfreeglut




QMAKE_RPATHDIR += $ORIGIN/lib

SOURCES += \
    RungeKutta4.cpp \


HEADERS += \

