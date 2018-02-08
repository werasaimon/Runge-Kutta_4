TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

LIBS += -lGL -lGLU -lX11

SOURCES += \
    RungeKutta4.cpp \
    freeglut/freeglut_xinput.c \
    freeglut/freeglut_window.c \
    freeglut/freeglut_videoresize.c \
    freeglut/freeglut_teapot.c \
    freeglut/freeglut_structure.c \
    freeglut/freeglut_stroke_roman.c \
    freeglut/freeglut_stroke_mono_roman.c \
    freeglut/freeglut_state.c \
    freeglut/freeglut_spaceball.c \
    freeglut/freeglut_overlay.c \
    freeglut/freeglut_misc.c \
    freeglut/freeglut_menu.c \
    freeglut/freeglut_main.c \
    freeglut/freeglut_joystick.c \
    freeglut/freeglut_input_devices.c \
    freeglut/freeglut_init.c \
    freeglut/freeglut_glutfont_definitions.c \
    freeglut/freeglut_geometry.c \
    freeglut/freeglut_gamemode.c \
    freeglut/freeglut_font_data.c \
    freeglut/freeglut_font.c \
    freeglut/freeglut_ext.c \
    freeglut/freeglut_display.c \
    freeglut/freeglut_cursor.c \
    freeglut/freeglut_callbacks.c

HEADERS += \
    freeglut/glut.h \
    freeglut/freeglut_teapot_data.h \
    freeglut/freeglut_internal.h \
    freeglut/GL/glut.h \
    freeglut/GL/freeglut_std.h \
    freeglut/GL/freeglut_ext.h \
    freeglut/GL/freeglut.h
