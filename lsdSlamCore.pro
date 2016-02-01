#-------------------------------------------------
#
# Project created by QtCreator 2015-10-29T00:13:56
#
#-------------------------------------------------

QT          -= core gui

CONFIG      += c++11

TARGET      = lsdSlamCore
TEMPLATE    = lib

DEFINES     += LSDSLAMCORE_LIBRARY

SOURCES +=  \
            src/LiveSLAMWrapper.cpp \
            src/SlamSystem.cpp

HEADERS +=  \
            src/LiveSLAMWrapper.h   \
            src/SlamSystem.h

unix {

    INCLUDEPATH += /home/sergey/MyProject/MySlamProject/Qt/
    INCLUDEPATH += /home/sergey/libs/Sophus

    INCLUDEPATH +=  /home/sergey/MyProject/MySlamProject/Qt/lsdSlamUtil/
    LIBS        +=  -L/home/sergey/MyProject/MySlamProject/Qt/FullProject/build/lsdSlamUtil    \
                    -llsdSlamUtil

    INCLUDEPATH +=  /home/sergey/MyProject/MySlamProject/Qt/lsdSlamIO/
    LIBS        +=  -L/home/sergey/MyProject/MySlamProject/Qt/FullProject/build/lsdSlamIO     \
                    -llsdSlamIO

    INCLUDEPATH += /home/sergey/MyProject/MySlamProject/Qt/lsdSlamFrame/
    LIBS        +=  -L/home/sergey/MyProject/MySlamProject/Qt/FullProject/build/lsdSlamFrame  \
                    -llsdSlamFrame

    INCLUDEPATH += /home/sergey/MyProject/MySlamProject/Qt/lsdSlamGlobalMapping/
    LIBS        +=  -L/home/sergey/MyProject/MySlamProject/Qt/FullProject/build/lsdSlamGlobalMapping  \
                    -llsdSlamGlobalMapping

    INCLUDEPATH += /home/sergey/MyProject/MySlamProject/Qt/lsdSlamTracking/
    LIBS        +=  -L/home/sergey/MyProject/MySlamProject/Qt/FullProject/build/lsdSlamTracking  \
                    -llsdSlamTracking

    INCLUDEPATH += /home/sergey/MyProject/MySlamProject/Qt/lsdSlam3DOutput/
    LIBS        +=  -L/home/sergey/MyProject/MySlamProject/Qt/FullProject/build/lsdSlam3DOutput  \
                    -llsdSlam3DOutput

    #INCLUDEPATH += /home/sergey/MyProject/MySlamProject/Qt/lsdSlamDepth/
    #LIBS        +=  -L/home/sergey/MyProject/MySlamProject/Qt/FullProject/build/lsdSlamDepth  \
    #                -llsdSlamDepth

    # OpenCV
    OPENCV_INCLUDE_PATH        = /home/sergey/libs/opencv-3.0.0/include
    OPENCV_INCLUDE_MODULE_PATH = /home/sergey/libs/opencv-3.0.0/release/modules

    OPENCV_LIBS_PATH           = /home/sergey/libs/opencv-3.0.0/release/lib

        message( " Unix - Version OpenCV - 3.00 - Release " )
        message( $$OPENCV_LIBS_PATH )

        LIBS    += -L$$OPENCV_LIBS_PATH
        LIBS    += -lopencv_objdetect   -lopencv_imgproc
        LIBS    += -lopencv_videoio     -lopencv_core
        LIBS    += -lopencv_imgcodecs   -lopencv_highgui
        LIBS    += -lopencv_features2d  -lopencv_calib3d

    # Boost
    LIBS    +=  -L/home/sergey/libs/boost_1_59_0/stage/lib      \
                -lboost_thread                                  \
                -lboost_system


    #target.path = /usr/lib
    target.path = /home/sergey/MyProject/MySlamProject/lsdSlamSharedLibs
    INSTALLS    +=  target
}
