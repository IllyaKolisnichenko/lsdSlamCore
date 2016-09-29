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

#include "LiveSLAMWrapper.h"

//#include "IOWrapper/TimestampedObject.h"
//#include "../lsdSlamIO/include/TimestampedObject.h"

//#include "IOWrapper/InputImageStream.h"
#include "../lsdSlamIO/include/InputImageStream.h"
//#include "IOWrapper/VideoReader/slamvideoreader.h"
#include "../lsdSlamIO/include/sources/videoreader.h"
#include "../lsdSlamIO/include/sources/imagereader.h"

#include "Output3DWrapper/myoutput3dwrapper.h"

//#include "IOWrapper/OpenCV/slamimagedisplay.h"

#include "SlamSystem.h"

namespace lsd_slam
{

LiveSLAMWrapper::LiveSLAMWrapper(const char* videoFilePath, const char* unditorFilePath) :
    m_bDoSlam   (false)
{
    // Module is not initialized yet
    isInitialized = false;

    m_pCVFaceCascade.load( FACE_CASCADE_NAME);
    /// It's in utils now
    // m_poImageDisplay = new SLAMImageDisplay();

    // Initialize the pointers
//    m_poImageStream  = new VideoReader( videoFilePath  );
    m_poImageStream  = new ImageReader( videoFilePath  );

    // Read the calibration file
    m_poImageStream->setCalibration( unditorFilePath );

    m_poImageStream->setFrameCallback((ImageStreamCallback*)this);

    // Set the parameters of the camera
    fx = m_poImageStream->fx();
    fy = m_poImageStream->fy();
    cx = m_poImageStream->cx();
    cy = m_poImageStream->cy();

    // Set the size of the image
    width   = m_poImageStream->width();
    height  = m_poImageStream->height();

    // Null the pointers
    outFile = nullptr;

    // Make a file name for the file that stores restored positions 
    // outFileName = packagePath+"estimated_poses.txt";
    outFileName = "estimated_poses.txt";

    // Initialize the matrix of the camera
    Sophus::Matrix3f K_sophus;
    K_sophus << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

    // Make Odometry
    // Create an instance of SlamSystem
    m_poMonoOdometry  = new SlamSystem( width, height, K_sophus, m_bDoSlam );

    // ROSOutput3DWrapper
//    m_poOutputWrapper =  new MyOutput3DWrapper(    m_poImageStream->width(),
//                                                   m_poImageStream->height()   );
    //  ???
    m_poMonoOdometry->setVisualization( m_poOutputWrapper );

    // Null the counter
    imageSeqNumber = 0;

    // Start to recieve Image
    m_poImageStream->run();
}

LiveSLAMWrapper::~LiveSLAMWrapper()
{
    /// CLEAR ALL POINTERS ?????

    if(m_poMonoOdometry != 0)
        delete m_poMonoOdometry;

	if(outFile != 0)
	{
		outFile->flush();
		outFile->close();
		delete outFile;
	}
}

void LiveSLAMWrapper::Loop()
{
    // The loop of threads processing
    while( true )
    {

        // Get the instance of an image from the buffer 
//        cv::Mat image = m_poImageStream->nextFrame();

//        if(image.empty())
//            break;
		
        /// We need a method for asynchronous reset, stop and pause
//        // If we need to reset the system (global variable) 
//        if( fullResetRequested )
//        {
//            // Reset everything 
//            resetAll();
//            // Reset the flag of request 
//            fullResetRequested = false;

//            // If the buffer is empty
//            if ( !(m_poImageStream->getBuffer()->size() > 0) )
//                // Go to the next iteration
//                continue;
//        }
		
        /// !!!! Вывести изображение
        /// //TODO:
 //       m_poImageDisplay->displayImage( "MyVideo", image.data );
        // Output the image
        Util::displayImage( "MyVideo", image );

        // Process a new image
        newImageCallback( image/*, image.timestamp*/ );

        //m_poImageDisplay->waitKey( 500 );
        //cv::waitKey( 20 );
	}
}

void LiveSLAMWrapper::newFrameCallback(  cv::Mat* frame )
{
    /// We need a method for asynchronous reset, stop and pause
//        // If we need to reset the system (global variable)
//        if( fullResetRequested )
//        {
//            // Reset everything
//            resetAll();
//            // Reset the flag of request
//            fullResetRequested = false;

//            // If the buffer is empty
//            if ( !(m_poImageStream->getBuffer()->size() > 0) )
//                // Go to the next iteration
//                continue;
//        }

    // Increment the counter 
	++ imageSeqNumber;

	// Convert image to grayscale, if necessary
    // Transform an image to greyscale
	cv::Mat grayImg;

    // Check the amount of canals
    if ( img.channels() == 1 )
        // Just assigning 
        grayImg = *frame;
	else
        // Transforming
        cvtColor( *frame, grayImg, CV_RGB2GRAY );
	
	// Assert that we work with 8 bit images
    assert( grayImg.elemSize() == 1 );
    assert( fx != 0 || fy != 0 );

    /// START SLAM !!!!!!!!!!!!
    // Need to initialize
    // Not initialized yet
    if( !isInitialized && m_poMonoOdometry != nullptr )
    {
        // Random initialization
        m_poMonoOdometry->randomInit( grayImg.data,/* imgTime.toSec()*/ 0.0, 1 );

        // Make a note of the initialization 
        isInitialized = true;
    }
    // If initialization is successful and SLAM pointer exists 
    else if( isInitialized && m_poMonoOdometry != nullptr )
    {
        m_poMonoOdometry->trackFrame(   grayImg.data    ,
                                        imageSeqNumber  ,
                                        false           ,
                                        /*imgTime.toSec()*/   0.0 );
    }

    /// !!!! Вывести изображение
    /// //TODO:
//       m_poImageDisplay->displayImage( "MyVideo", image.data );
    // Output the image
    Util::displayImage( "MyVideo", image );

    // Process a new image
    newImageCallback( image/*, image.timestamp*/ );

    //m_poImageDisplay->waitKey( 500 );
    //cv::waitKey( 20 );
}

/*
//void LiveSLAMWrapper::logCameraPose(const SE3& camToWorld, double time)
//{
//    Sophus::Quaternionf quat    = camToWorld.unit_quaternion().cast<float>();
//    Eigen::Vector3f     trans   = camToWorld.translation().cast<float>();

//	char buffer[1000];
//	int num = snprintf(buffer, 1000, "%f %f %f %f %f %f %f %f\n",
//			time,
//			trans[0],
//			trans[1],
//			trans[2],
//			quat.x(),
//			quat.y(),
//			quat.z(),
//			quat.w());

//	if(outFile == 0)
//		outFile = new std::ofstream(outFileName.c_str());
//	outFile->write(buffer,num);
//	outFile->flush();
//}
*/

void LiveSLAMWrapper::requestReset()
{
    fullResetRequested = true;

        /// TODO:
//    notifyCondition.notify_all(); ???????????
}

void LiveSLAMWrapper::resetAll()
{
    if(m_poMonoOdometry != nullptr)
    {
        delete m_poMonoOdometry;
        printf("Deleted SlamSystem Object!\n");

        Sophus::Matrix3f K;
        K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

        m_poMonoOdometry = new SlamSystem(width,height, K, m_bDoSlam);

        m_poMonoOdometry->setVisualization(m_poOutputWrapper);

    }
    imageSeqNumber  = 0;
    isInitialized   = false;

    Util::closeAllWindows();
}

void LiveSLAMWrapper::detectAndDraw(cv::Mat &image )
{
    // Список прямоугольников для лиц
    std::vector < cv::Rect >   faces;
    // Временный фрейм
    cv::Mat             frame_gray;

    // Преобразовать цвета
    //cv::cvtColor( image, frame_gray, CV_RGB2GRAY  );

    frame_gray = image.clone();

    // Выровнять гистограмму
    cv::equalizeHist( frame_gray, frame_gray );

    // Определить список лиц
    m_pCVFaceCascade.detectMultiScale(  frame_gray              ,
                                        faces                   ,
                                        1.1                     ,
                                        2                       ,
                                        0|CV_HAAR_SCALE_IMAGE   ,
                                        cv::Size(100, 100)      );

    // qDebug() << "Faces number: " << faces.size();

    if( faces.size() != 0)
    {
        //m_oTempImage = m_oCVMat.clone();
        cv::Mat m_oTempImage = frame_gray.clone();

//        cv::imshow("Grayscale Image", frame_gray);
        frame_gray.setTo(cv::Scalar(0, 0, 0));

        m_oTempImage = m_oTempImage(faces[0]);

//        cv::imshow("Before equalize Image", m_oTempImage);
        // Выровнять гистограмму
        cv::equalizeHist( m_oTempImage, m_oTempImage );

//        cv::imshow("After equalize Image", m_oTempImage);

        // Преобразовать цвета
        //cv::cvtColor( m_oTempImage, TempImage_gray, CV_RGB2GRAY  );

        m_oTempImage.copyTo( frame_gray(faces[0]) );

//        cv::imshow("Result Image", frame_gray);
    }

    image = frame_gray.clone();
}

}
