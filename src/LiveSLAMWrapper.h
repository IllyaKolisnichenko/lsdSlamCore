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

#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>

#include "IOWrapper/NotifyBuffer.h"
#include "ImageDisplay.h"

#include "SophusUtil.h"

#include "opencv2/opencv.hpp"

namespace cv {
	class Mat;
}

namespace lsd_slam
{
#define FACE_CASCADE_NAME   "/home/sergey/libs/opencv-3.0.0/data/haarcascades/haarcascade_frontalface_alt.xml"


class SlamSystem;
class Output3DWrapper;

class InputImageStream;
//class SLAMImageDisplay;
class Timestamp;

/** Wrapper for SlamSystem */
class LiveSLAMWrapper
{

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
     * @brief Loop
     *
     * Runs the main processing loop. Will never return.
     */
	void Loop();
	
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

    /**
     * @brief newImageCallback
     *
     * Callback function for new RGB images.
     *
     * @param img
     * @param imgTime
     */
	void newImageCallback(const cv::Mat& img, Timestamp imgTime);

//	/** Writes the given time and pose to the outFile. */
//	void logCameraPose(const SE3& camToWorld, double time);
	
    // Returns the pointer to the SlamSystem object
//    inline SlamSystem* getSlamSystem() {return m_poMonoOdometry;}

    void detectAndDraw(cv::Mat &image);
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
//    cv::CascadeClassifier   m_pCVFaceCascade;
};

}
