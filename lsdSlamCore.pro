#-------------------------------------------------
#
# Project created by QtCreator 2015-10-29T00:13:56
#
#-------------------------------------------------

QT          += core gui
QT          += widgets

CONFIG      += c++11

TARGET      = lsdSlamCore
TEMPLATE    = lib

DEFINES     += LSDSLAMCORE_LIBRARY

#QMAKE_CFLAGS_DEBUG    += -pg
#QMAKE_CXXFLAGS_DEBUG  += -pg
#QMAKE_LFLAGS_DEBUG    += -pg

INCLUDEPATH += $$PWD/include

SOURCES +=  \
            src/lsdslamoutput.cpp               \
            src/SlamSystem.cpp                  \
            src/lsdslamoutputstorage.cpp \
    lsdslamoutputdefault.cpp

HEADERS +=  \
            include/lsdslamoutput.h             \
            include/SlamSystem.h                \
            include/lsdslamoutputstorage.h \
    lsdslamoutputdefault.h

unix {

    BASE_LIBS_PATH = $$PWD/../build

    INCLUDEPATH +=  ../lsdSlamUtil/
    LIBS        +=  -L$$BASE_LIBS_PATH/lsdSlamApp       \
                    -llsdSlamUtil

    INCLUDEPATH +=  ../lsdSlamIO/
    LIBS        +=  -L$$BASE_LIBS_PATH/lsdSlamApp       \
                    -llsdSlamIO

    INCLUDEPATH +=  ../lsdSlamFrame/
    LIBS        +=  -L$$BASE_LIBS_PATH/lsdSlamApp       \
                    -llsdSlamFrame

    INCLUDEPATH +=  ../lsdSlamGlobalMapping/
    LIBS        +=  -L$$BASE_LIBS_PATH/lsdSlamApp       \
                    -llsdSlamGlobalMapping

    INCLUDEPATH +=  ../lsdSlamTracking/
    LIBS        +=  -L$$BASE_LIBS_PATH/lsdSlamApp       \
                    -llsdSlamTracking

    # OpenCV
    LIBS    += -L/usr/local/lib
    LIBS    += -lopencv_objdetect   -lopencv_imgproc
    LIBS    += -lopencv_videoio     -lopencv_core
    LIBS    += -lopencv_imgcodecs   -lopencv_highgui

    # Boost
    LIBS    +=  -lboost_thread -lboost_system

    target.path = $$BASE_LIBS_PATH/lsdSlamApp
    INSTALLS += target
}
