#include "lsdslamoutputstorage.h"

#include <inttypes.h>

#include "include/SophusUtil.h"

#include "DataStructures/Frame.h"
#include "KeyFrameGraph.h"

lsdSlamOutputStorage::lsdSlamOutputStorage()
{
//    m_sOutputDir = "output/";
    // Estimated camera positons file
    m_sPoseFileName = "/home/sergey/MyLsdSlamProject/TestData/tempCoods.txt";

    setOutputDir( "slam_output/" );

    // File stream for estimated camera position data
    m_pPosFileStream = new std::ofstream( m_sPoseFileName );
    // First Frame
    *m_pPosFileStream << "1 0 0 0 1 0 0 0" << std::endl;

    // Estimated camera positons file
    char* keyframeFileName = "/home/sergey/MyLsdSlamProject/TestData/keyframesCoods.txt";
    // File stream for estimated camera position data
    m_pKeyframesFileStream = new std::ofstream( keyframeFileName );

    // Point cloud dir path
//    m_pXyzDirPath = "/home/sergey/MyLsdSlamProject/TestData/KeyFrame";
    // Set path to dir
//    xyzFilename       = m_pXyzDirPath;

    xyzFilename = "/home/sergey/MyLsdSlamProject/TestData/KeyFrame";
    // Set template
    xyzFilenameFormat = "%s_%010"PRIu32".xyz";
    // Mesuring buffer length
    int max_name_len = snprintf( NULL,
                                 0,
                                 xyzFilenameFormat,
                                 xyzFilename,
                                 UINT32_MAX         );

    // Save temporary
    m_pXyzFileNamePart = xyzFilename;
    // Reserve buffer
    xyzFilename = (char*)calloc( max_name_len + 1, sizeof(char) );

    // Copy to buffer
    strcpy( xyzFilename, m_pXyzFileNamePart );
}

lsdSlamOutputStorage::~lsdSlamOutputStorage()
{
    m_pPosFileStream->flush();
    m_pPosFileStream->close();
}

void lsdSlamOutputStorage::setOutputDir(const char *path)
{
//    std::string = path;

//    // Estimated camera positons file
//    m_sPoseFileName = path + "allCameraPos.txt";

//    // Estimated camera positons file
//    char* keyframeFileName = path + "kfCameraPos.txt";

//    xyzFilename = path + "KeyFrames/";
}

void lsdSlamOutputStorage::publishKeyframeGraph( lsd_slam::KeyFrameGraph *graph )
{

}

void lsdSlamOutputStorage::publishKeyframe( lsd_slam::Frame *kf)
{
    /// ??? Level number - check in original
    int publishLvl = 0;

    // Make new file name
    sprintf( xyzFilename, xyzFilenameFormat, m_pXyzFileNamePart, kf->id() );

    // open file stream
    std::ofstream* pXyzStream = new std::ofstream( xyzFilename );

    // Lock data
    boost::shared_lock<boost::shared_mutex> lock = kf->getActiveLock();

    *pXyzStream << kf->id() << std::endl;

    int wight = kf->width ( publishLvl );
    int hight = kf->height( publishLvl );

    *pXyzStream << wight << " " << hight << std::endl;

    // Copy camera params
    *pXyzStream << kf->fx( publishLvl ) << " ";
    *pXyzStream << kf->fy( publishLvl ) << " ";
    *pXyzStream << kf->cx( publishLvl ) << " ";
    *pXyzStream << kf->cy( publishLvl ) << " ";

    *pXyzStream << std::endl;

    /*
    // Base solution for camera position
    // Copy 7 numbers of camToWorld.data ( KF camera position )
//    float* tempBuff = kf->getScaledCamToWorld().cast<float>().data();

//    for(int i = 0; i < 7; i++)
//    {
//        *pXyzStream << tempBuff[i] << " ";
//    }

//    *pXyzStream << std::endl;

*/

    // Temp Solution for translation
    double x, y, z, w;

    SE3 camToWorld = se3FromSim3( kf->getScaledCamToWorld() );

    x = camToWorld.translation()[0];
    y = camToWorld.translation()[1];
    z = camToWorld.translation()[2];

    *pXyzStream             << " " << x << " " << y << " " << z;

    *m_pKeyframesFileStream << kf->id() << " " << x << " " << y << " " << z;

    x = camToWorld.so3().unit_quaternion().x();
    y = camToWorld.so3().unit_quaternion().y();
    z = camToWorld.so3().unit_quaternion().z();
    w = camToWorld.so3().unit_quaternion().w();

    if (  wight < 0)
    {
        x *= -1;
        y *= -1;
        z *= -1;
        w *= -1;
    }

    *pXyzStream             <<  " " << w << " " << x << " " << y << " " << z << std::endl;

    *m_pKeyframesFileStream <<  " " << w << " " << x << " " << y << " " << z << std::endl;

    // Depth INFO save to file
    const float* idepth     = kf->idepth     (publishLvl);
    const float* idepthVar  = kf->idepthVar  (publishLvl);
    const float* color      = kf->image      (publishLvl);

    for( int idx = 0; idx < wight * hight; idx++ )
    {
        *pXyzStream << idepth    [idx] << " ";
        *pXyzStream << idepthVar [idx] << " ";

        *pXyzStream << color[idx] << " ";
        *pXyzStream << color[idx] << " ";
        *pXyzStream << color[idx] << " ";
        *pXyzStream << color[idx] << " ";

        *pXyzStream << std::endl;
    }

//    printf( "INFO: PC with points was sended! %zu \n", w*h );

    // close file stream
    pXyzStream->flush();
    pXyzStream->close();
}

void lsdSlamOutputStorage::publishTrackedFrame( lsd_slam::Frame *kf)
{
    double x, y, z, w;

    SE3 camToWorld = se3FromSim3( kf->getScaledCamToWorld() );

    x = camToWorld.translation()[0];
    y = camToWorld.translation()[1];
    z = camToWorld.translation()[2];

    *m_pPosFileStream << kf->id() << " " << x << " " << y << " " << z;

    x = camToWorld.so3().unit_quaternion().x();
    y = camToWorld.so3().unit_quaternion().y();
    z = camToWorld.so3().unit_quaternion().z();
    w = camToWorld.so3().unit_quaternion().w();

    if (  w < 0)
    {
        x *= -1;
        y *= -1;
        z *= -1;
        w *= -1;
    }

    *m_pPosFileStream << " " << w << " " << x << " " << y  << " " << z << std::endl;

    //    pMsg.header.stamp       = kf->timestamp();
    //    pMsg.header.frame_id    = "world";

}

void lsdSlamOutputStorage::publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
{

}

void lsdSlamOutputStorage::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
{

}

void lsdSlamOutputStorage::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1> > trajectory, std::string identifier)
{

}
