/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

//#include <QMainWindow>
//#include <QLabel>
//#include <QImage>

#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>

#include "ImageDisplay.h"

#include "SophusUtil.h"

#include <opencv2/opencv.hpp>

#include <imagestreamcallback.h>

namespace cv {
	class Mat;
}

class InputImageStream;

namespace lsd_slam
{

#define FACE_CASCADE_NAME   "/home/sergey/libs/opencv-3.0.0/data/haarcascades/haarcascade_frontalface_alt.xml"

class SlamSystem;
class Output3DWrapper;

//class SLAMImageDisplay;
//class Timestamp;

/** Wrapper for SlamSystem */
class LiveSLAMWrapper :/* public QMainWindow, */public ImageStreamCallback
{

//friend class LiveSLAMWrapperROS;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief LiveSLAMWrapper
     *
     * Constructor.
     *
     * @param videoFilePath
     * @param unditorFilePath
     */
    LiveSLAMWrapper(const char* videoFilePath, const char* unditorFilePath);

	/** Destructor. */
	~LiveSLAMWrapper();
		
    /**
     * @brief run
     *
     * Runs the main processing loop. Will never return.
     */
    void run();

    /**
     * @brief run
     *
     * Runs the main processing loop. Will never return.
     */
    void join();
	
    /**
     * @brief requestReset
     *
     * Requests a reset from a different thread.
     */
	void requestReset();
	
    /**
     * @brief resetAll
     *
     * Resets everything, starting the odometry from the beginning again.
     */
    void resetAll();

//	/** Writes the given time and pose to the outFile. */
//	void logCameraPose(const SE3& camToWorld, double time);
	
    // Returns the pointer to the SlamSystem object
//    inline SlamSystem* getSlamSystem() {return m_poMonoOdometry;}

    void detectAndDraw(cv::Mat &image);

protected:
    /**
     * @brief newFrameCallback
     *
     * Callback function for new RGB images.
     *
     * @param img
     *
     */
    void  newFrameCallback( cv::Mat* frame);

private:
    /// Pointer to the object of input thread
    InputImageStream*   m_poImageStream;
    /// Pointer to the object of output thread
    Output3DWrapper*    m_poOutputWrapper;

    // Object for Image Dispaly
 //   SLAMImageDisplay*   m_poImageDisplay;

    /// Initialization stuff
    bool    isInitialized;

    bool    m_bDoSlam;

    /// MonoOdometry
    SlamSystem*     m_poMonoOdometry;

    std::string     outFileName;
    std::ofstream*  outFile;

    float   fx, fy, cx, cy;
    int     width, height;

    int     imageSeqNumber;

    // Классификаторы для обнраужения объекта
    cv::CascadeClassifier   m_pCVFaceCascade;

    cv::Mat m_tempImage;

//    QLabel  m_imageLabel;
};

}
