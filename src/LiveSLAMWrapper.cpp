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

#include "IOWrapper/TimestampedObject.h"

//#include "IOWrapper/InputImageStream.h"
#include "IOWrapper/VideoReader/slamvideoreader.h"

#include "Output3DWrapper/myoutput3dwrapper.h"

//#include "IOWrapper/OpenCV/slamimagedisplay.h"

#include "src/SlamSystem.h"

namespace lsd_slam
{

LiveSLAMWrapper::LiveSLAMWrapper(const char* videoFilePath, const char* unditorFilePath) :
    m_bDoSlam   (false)
{
    // Модуль еще не инициализирован
    isInitialized = false;

    m_pCVFaceCascade.load( FACE_CASCADE_NAME);
    /// It's in utils now
    // m_poImageDisplay = new SLAMImageDisplay();

    // Инициализровать указатели
    m_poImageStream  = new SLAMVideoReader( 8, videoFilePath  );

    // Вычитать файл каллибровки
    m_poImageStream->setCalibration( unditorFilePath );

    // Установить параметры камеры
    fx = m_poImageStream->fx();
    fy = m_poImageStream->fy();
    cx = m_poImageStream->cx();
    cy = m_poImageStream->cy();

    // Установить размеры изображения
    width   = m_poImageStream->width();
    height  = m_poImageStream->height();

    // Обнулить указатель
    outFile = nullptr;
    // Сформировать название файла для хранения восстановленных положений
    // outFileName = packagePath+"estimated_poses.txt";
    outFileName = "estimated_poses.txt";

    // Инициализировать матрицу камеры
    Sophus::Matrix3f K_sophus;
    K_sophus << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

    /// make Odometry
    // Создаем экземпляр SlamSystem
    m_poMonoOdometry  = new SlamSystem( width, height, K_sophus, m_bDoSlam );

    // ROSOutput3DWrapper
    m_poOutputWrapper =  new MyOutput3DWrapper(    m_poImageStream->width(),
                                                   m_poImageStream->height()   );
    //  ???
    m_poMonoOdometry->setVisualization( m_poOutputWrapper );

    // Сбросить счетчик
    imageSeqNumber = 0;

    // Start recieve Image
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
    // Цикл обработки потоков
    while( true )
    {

        // Получить экземпляр изображения из буффера
        TimestampedMat image = m_poImageStream->next();

        if(image.data.empty())
            break;
		
        /// Необдим способ асинхронного сброса, остановки, паузы
//        // Если требуется сбросить систему (глобальная переменная)
//        if( fullResetRequested )
//        {
//            // Сбросить все
//            resetAll();
//            // СБросить флаг запроса
//            fullResetRequested = false;

//            // Если буфер пустой
//            if ( !(m_poImageStream->getBuffer()->size() > 0) )
//                // Перейти на следующую итерацию
//                continue;
//        }
		
        /// !!!! Вывести изображение
        /// //TODO:
 //       m_poImageDisplay->displayImage( "MyVideo", image.data );
        // Вывести изображение

        detectAndDraw( image.data );

        Util::displayImage( "MyVideo", image.data );

        //Обработать новое изображение
        newImageCallback( image.data, image.timestamp );

        //m_poImageDisplay->waitKey( 500 );
        // cv::waitKey( 100 );
	}
}

void LiveSLAMWrapper::newImageCallback( const cv::Mat& img, Timestamp imgTime )
{
    // Увеличить счетчик
	++ imageSeqNumber;

	// Convert image to grayscale, if necessary
    /// **** Я ЭТО СДЕЛАЛ НА УРОВНЕ КАМЕРЫ !!!!!!!!! *********
    // Преобразовать изображение в чернобелое
	cv::Mat grayImg;

    // Проверить количество каналов
    if ( img.channels() == 1 )
        // Просто присваиваем
		grayImg = img;
	else
        // Преобразовуем
        cvtColor( img, grayImg, CV_RGB2GRAY );
	
	// Assert that we work with 8 bit images
    assert( grayImg.elemSize() == 1 );
    assert( fx != 0 || fy != 0 );

    /// START SLAM !!!!!!!!!!!!
    // need to initialize
    // Еще не инициализирован
    if( !isInitialized && m_poMonoOdometry != nullptr )
    {
        // Случайная инициализация
        m_poMonoOdometry->randomInit( grayImg.data, imgTime.toSec(), 1 );

        // Отметить инициализацию
        isInitialized = true;
    }
    // Если прошла инициализация и указатель на SLAM существует
    else if( isInitialized && m_poMonoOdometry != nullptr )
    {
        m_poMonoOdometry->trackFrame(   grayImg.data    ,
                                        imageSeqNumber  ,
                                        false           ,
                                        imgTime.toSec()     );
    }
}

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
