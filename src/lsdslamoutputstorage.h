#ifndef LSDSLAMOUTPUTSTORAGE_H
#define LSDSLAMOUTPUTSTORAGE_H

#include <string>
#include <fstream>

#include "include/SophusUtil.h"

#include "lsdslamoutput.h"

class lsdSlamOutputStorage : public lsdSlamOutput
{
public:
    virtual ~lsdSlamOutputStorage();

    // Set output path
    void setOutputDir( const char* path );

    virtual void publishKeyframeGraph( lsd_slam::KeyFrameGraph* graph );

    /**
     * @brief publishKeyframe
     *
     * Publishes a keyframe. if that frame already existis, it is overwritten, otherwise it is added.
     * @param kf
     */
    virtual void publishKeyframe(lsd_slam::Frame* kf);

    /**
     * @brief publishTrackedFrame
     *
     * Published a tracked frame that did not become a keyframe (yet; i.e. has no depth data)
     * @param kf
     */
    virtual void publishTrackedFrame( lsd_slam::Frame* kf );

    /**
     * @brief publishTrajectory
     *
     * Publishes graph and all constraints, as well as updated KF poses.
     * @param trajectory
     * @param identifier
     */
    virtual void publishTrajectory          ( std::vector< Eigen::Matrix< float, 3, 1 > >   trajectory,
                                              std::string                                   identifier  );

    virtual void publishTrajectoryIncrement ( Eigen::Matrix< float, 3, 1 >  pt,
                                                std::string                 identifier  );

    virtual void publishDebugInfo(Eigen::Matrix<float, 20, 1> data);

protected:
    lsdSlamOutputStorage();

    friend class lsdSlamOutput;

private:
    // Base path to otput dir
//    std::string     m_sOutputDir;

    std::string     m_sPoseFileName;

    // Temp solution - path coods file
    std::ofstream*  m_pPosFileStream;

    // Temp solution - keyframes coods file
    std::ofstream*  m_pKeyframesFileStream;

    // Temp solution Point Cloud coods
    char* m_pXyzDirPath;
    char* m_pXyzFileNamePart;
    char* xyzFilename;
    char* xyzFilenameFormat;
};

#endif // LSDSLAMOUTPUTSTORAGE_H
