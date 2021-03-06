#-------------------------------------------------
#
# Project created by QtCreator 2015-10-29T00:13:56
#
#-------------------------------------------------

QT          -= core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG      += c++11

TARGET      = lsdSlamCore
TEMPLATE    = lib

DEFINES     += LSDSLAMCORE_LIBRARY

QMAKE_CFLAGS_DEBUG    += -g -funwind-tables -fno-omit-frame-pointer -std=c++11
QMAKE_CXXFLAGS_DEBUG  += -g -funwind-tables -fno-omit-frame-pointer -std=c++11
QMAKE_LFLAGS_DEBUG    += -g -funwind-tables -fno-omit-frame-pointer -std=c++11

#QMAKE_CFLAGS   += -std=c++11 -fopenmp
#QMAKE_CXXFLAGS += -std=c++11 -fopenmp
#QMAKE_LFLAGS   += -std=c++11 -fopenmp

SOURCES +=  \
            LiveSLAMWrapper.cpp \
            SlamSystem.cpp

HEADERS +=  \
            LiveSLAMWrapper.h   \
            SlamSystem.h

unix {

#    INCLUDEPATH += /home/sergey/MyProject/MySlamProject/Qt/
#    INCLUDEPATH += /home/sergey/libs/Sophus

    BASE_LIBS_PATH = $$PWD/../build

    INCLUDEPATH +=  ../lsdSlamUtil/
#    LIBS        +=  -L$$BASE_LIBS_PATH/lsdSlamUtil          \
#                    -llsdSlamUtil

    INCLUDEPATH +=  ../lsdSlamIO/
    LIBS        +=  -L$$BASE_LIBS_PATH/lsdSlamIO            \
                    -llsdSlamIO

    INCLUDEPATH +=  ../lsdSlamFrame/
    LIBS        +=  -L$$BASE_LIBS_PATH/lsdSlamFrame         \
                    -llsdSlamFrame

    INCLUDEPATH +=  ../lsdSlamGlobalMapping/
    LIBS        +=  -L$$BASE_LIBS_PATH/lsdSlamGlobalMapping \
                    -llsdSlamGlobalMapping

    INCLUDEPATH +=  ../lsdSlamTracking/
    LIBS        +=  -L$$BASE_LIBS_PATH/lsdSlamTracking      \
                    -llsdSlamTracking

    INCLUDEPATH +=  ../lsdSlam3DOutput/
#    LIBS        +=  -L$$BASE_LIBS_PATH/lsdSlam3DOutput      \
#                    -llsdSlam3DOutput

    # OpenCV
#    OPENCV_INCLUDE_PATH        = /home/sergey/libs/opencv-3.0.0/include
#    OPENCV_INCLUDE_MODULE_PATH = /home/sergey/libs/opencv-3.0.0/release/modules

#    OPENCV_LIBS_PATH           = /home/sergey/libs/opencv-3.0.0/release/lib

#        message( " Unix - Version OpenCV - 3.00 - Release " )
#        message( $$OPENCV_LIBS_PATH )

##    LIBS    += -L$$OPENCV_LIBS_PATH
    LIBS    += -L/usr/local/lib

    LIBS    += -lopencv_objdetect   -lopencv_imgproc
    LIBS    += -lopencv_videoio     -lopencv_core
    LIBS    += -lopencv_highgui
#    LIBS    += -lopencv_features2d  -lopencv_calib3d
#    LIBS    += -lopencv_imgcodecs
    # Boost
    LIBS    +=  -lboost_thread -lboost_system

    #target.path = /usr/lib
    target.path = $$BASE_LIBS_PATH/lsdSlamApp
    INSTALLS += target
}
