#ifndef LSDSLAMOUTPUTDEFAULT_H
#define LSDSLAMOUTPUTDEFAULT_H

#include "lsdslamoutput.h"

class LsdSlamOutputDefault : public lsdSlamOutput
{
public:
    virtual ~LsdSlamOutputDefault();

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

public:
//    LsdSlamOutputDefault();
    friend class lsdSlamOutput;

};

#endif // LSDSLAMOUTPUTDEFAULT_H
