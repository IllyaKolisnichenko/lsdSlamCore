#include "lsdslamoutputdefault.h"

//LsdSlamOutputDefault::LsdSlamOutputDefault()
//{

//}

LsdSlamOutputDefault::~LsdSlamOutputDefault()
{

}

void LsdSlamOutputDefault::publishKeyframeGraph( lsd_slam::KeyFrameGraph *graph )
{
}

void LsdSlamOutputDefault::publishKeyframe( lsd_slam::Frame *kf )
{
    /*
    /// ??? Level number - check in original
    int publishLvl = 0;

    // Make new file name
    sprintf( xyzFilename, xyzFilenameFormat, xyzFileNamePart, kf->id() );

    // open file stream
    std::ofstream* pXyzStream = new std::ofstream( xyzFilename );

    // Lock data
    boost::shared_lock<boost::shared_mutex> lock = kf->getActiveLock();

    *pXyzStream << kf->id() << std::endl;

    int w = kf->width ( publishLvl );
    int h = kf->height( publishLvl );

    *pXyzStream << w << " " << h << std::endl;

    // Copy camera params
    *pXyzStream << kf->fx( publishLvl ) << " ";
    *pXyzStream << kf->fy( publishLvl ) << " ";
    *pXyzStream << kf->cx( publishLvl ) << " ";
    *pXyzStream << kf->cy( publishLvl ) << " ";

    *pXyzStream << std::endl;

    // Base solution for camera position
    // Copy 7 numbers of camToWorld.data ( KF camera position )
    float* tempBuff = kf->getScaledCamToWorld().cast<float>().data();

    for(int i = 0; i < 7; i++)
    {
        *pXyzStream << tempBuff[i] << " ";
    }

    *pXyzStream << std::endl;

    // Temp Solution for translation
    SE3 camToWorld = se3FromSim3( kf->getScaledCamToWorld() );

    double x = camToWorld.translation()[0];
    double y = camToWorld.translation()[1];
    double z = camToWorld.translation()[2];

    *pXyzStream << kf->id() << " " << x << " " << y << " " << z << " " <<  "\n";

    // Depth INFO save to file
    const float* idepth     = kf->idepth     (publishLvl);
    const float* idepthVar  = kf->idepthVar  (publishLvl);
    const float* color      = kf->image      (publishLvl);

    for( int idx = 0; idx < w * h; idx++ )
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
    */
}

void LsdSlamOutputDefault::publishTrackedFrame( lsd_slam::Frame *kf )
{
//    SE3 camToWorld = se3FromSim3( kf->getScaledCamToWorld() );

//    double x = camToWorld.translation()[0];
//    double y = camToWorld.translation()[1];
//    double z = camToWorld.translation()[2];

//    *pCoodsStream << kf->id() << " " << x << " " << y << " " << z << " " <<  "\n";


    //    pMsg.pose.orientation.x = camToWorld.so3().unit_quaternion().x();
    //    pMsg.pose.orientation.y = camToWorld.so3().unit_quaternion().y();
    //    pMsg.pose.orientation.z = camToWorld.so3().unit_quaternion().z();
    //    pMsg.pose.orientation.w = camToWorld.so3().unit_quaternion().w();

    //    if (pMsg.pose.orientation.w < 0)
    //    {
    //        pMsg.pose.orientation.x *= -1;
    //        pMsg.pose.orientation.y *= -1;
    //        pMsg.pose.orientation.z *= -1;
    //        pMsg.pose.orientation.w *= -1;
    //    }

    //    pMsg.header.stamp       = kf->timestamp();
    //    pMsg.header.frame_id    = "world";

}

void LsdSlamOutputDefault::publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
{

}

void LsdSlamOutputDefault::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
{

}

void LsdSlamOutputDefault::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1> > trajectory, std::string identifier)
{

}
