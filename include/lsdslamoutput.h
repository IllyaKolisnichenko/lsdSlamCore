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

#include <string>

#include "include/SophusUtil.h"

//class cv::Mat;
class KeyFrameGraph;

using namespace lsd_slam;

namespace lsd_slam
{
    class Frame;
    class KeyFrameGraph;
}

/** A class. Output3DWrapper. Virtual 3D display object. */
class lsdSlamOutput
{
public:
    enum LsdSlamOutput_ID { Default_ID = 0, Storage_ID };

public:
//    lsdSlamOutput();
    virtual ~lsdSlamOutput(){}

    virtual void publishKeyframeGraph( lsd_slam::KeyFrameGraph* graph ) = 0;

    /**
     * @brief publishKeyframe
     *
     * Publishes a keyframe. if that frame already existis, it is overwritten, otherwise it is added.
     * @param kf
     */
    virtual void publishKeyframe    ( lsd_slam::Frame* kf) = 0;

    /**
     * @brief publishTrackedFrame
     *
     * Published a tracked frame that did not become a keyframe (yet; i.e. has no depth data)
     * @param kf
     */
    virtual void publishTrackedFrame( lsd_slam::Frame* kf ) = 0;

    /**
     * @brief publishTrajectory
     *
     * Publishes graph and all constraints, as well as updated KF poses.
     * @param trajectory
     * @param identifier
     */
    virtual void publishTrajectory          ( std::vector< Eigen::Matrix< float, 3, 1 > >   trajectory,
                                              std::string                                   identifier  ) = 0;

    virtual void publishTrajectoryIncrement ( Eigen::Matrix< float, 3, 1 >  pt,
                                                std::string                 identifier  ) = 0;

    virtual void publishDebugInfo(Eigen::Matrix<float, 20, 1> data) = 0;

    static lsdSlamOutput* create( LsdSlamOutput_ID id = Default_ID );

};
