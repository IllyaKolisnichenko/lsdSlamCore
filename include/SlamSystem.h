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

#include <vector>

#include <boost/thread.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/locks.hpp>

#include "include/settings.h"
#include "include/SophusUtil.h"

#include "opencv2/core/core.hpp"

#include "Relocalizer.h"

class DebugImage;
class lsdSlamOutput;

namespace lsd_slam
{

class   TrackingReference;

class   KeyFrameGraph;
class   TrackableKeyFrameSearch;
struct  KFConstraintStruct;

class   SE3Tracker;
class   Sim3Tracker;

class   DepthMap;
class   Frame;
//class   FramePoseStruct;

//class   DataSet;

typedef Eigen::Matrix<float, 7, 7> Matrix7x7;

class SlamSystem
{
friend class IntegrationTest;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//******************************************************************************
//****************** Конструктор / Дисруктор ***********************************

    /**
     * @brief SlamSystem
     *
     * Constructor.
     */
    SlamSystem( int w, int h, Eigen::Matrix3f K, bool enableSLAM = true );

    /**
     * @brief SlamSystem
     *
     * Constructor.
     */
    SlamSystem( const SlamSystem& ) = delete;
    SlamSystem& operator=(const SlamSystem&) = delete;

    /**
     * @brief ~SlamSystem()
     *
     * Destructor.
     */
    ~SlamSystem();

private:

    /// Used only before destruction to notify threads to finish.
    bool keepRunning;

//******************************************************************************
//*********** Функиции инициализации / завершения ******************************

public:

    /**
     * @brief Initial initialization.
     * @param image
     * @param timeStamp
     * @param id
     */
    void randomInit ( uchar* image, double timeStamp, int id );

    // Почиму-то нигде не используется ( может быть для обработки изиображений ?? )
//    void gtDepthInit( uchar* image, float* depth, double timeStamp, int id );

    // Завершает всю работу перед выходом
    // finalizes the system, i.e. blocks and does all remaining loop-closures etc.
//    void finalize();

    /**
     * @brief Sets the visualization where point clouds and camera poses will be sent to.
     * @param outputWrapper
     */
    void setVisualization(lsdSlamOutput* pOutputWrapper );

private:

//******************************************************************************
//************************ Работа с KayFrame ***********************************

public:
    /**
     * @brief trackFrame
     *
     *  Called after receieving a new frame
     *  Tracks a frame,
     *  First frame will return Identity = camToWord,
     *  Returns camToWord transformation of the tracked frame,
     *  frameID needs to be monotonically increasing.
     *
     * @param image
     * @param frameID
     * @param blockUntilMapped
     * @param timestamp
     */
    void trackFrame(    uchar*          image               ,
                        unsigned int    frameID             ,
                        bool            blockUntilMapped    ,
                        double          timestamp               );

    /**
     * @brief getCurrentKeyframe
     * @warning (Not thread-safe)
     * @return Returns the pointer to the current frame
     */
    inline Frame* getCurrentKeyframe() { return m_pCurrentKeyFrame.get();}

private:
    void finishCurrentKeyframe();
    void discardCurrentKeyframe();

    bool updateKeyframe();

    void changeKeyframe(bool noCreate, bool force, float maxScore);

    void createNewCurrentKeyframe(std::shared_ptr<Frame> newKeyframeCandidate);

    void loadNewCurrentKeyframe(Frame* keyframeToLoad);

private:

    /// SET & READ EVERYWHERE. Changed (and, for VO, maybe deleted). Only by Mapping thread within exclusive lock.
    std::shared_ptr<Frame>      m_pCurrentKeyFrame;

    /// Only used in odometry-mode, to keep a Key Frame alive until it is deleted. Only accessed within currentKeyFrameMutex lock.
    std::shared_ptr<Frame>      trackingReferenceFrameSharedPT;
    boost::mutex                currentKeyFrameMutex;

    /// Pushed in tracking, READ & CLEARED in mapping. Array of "unattached" frames.
    std::deque< std::shared_ptr<Frame> >    unmappedTrackedFrames;

    /// Mutex to protect these frames.
    boost::mutex                            unmappedTrackedFramesMutex;

    /// Notifies about some changes.
    boost::condition_variable               unmappedTrackedFramesSignal;

    //**************************************************************************
    //************************ Работа с Mapping ********************************

public:

    /**
     * @brief Main Mapping function
     *
     * @return
     */
    bool doMappingIteration();

private:

    /**
     * @brief mappingThreadLoop
     *
     * Function of mapping thread.
     */
    void mappingThreadLoop();

    void debugDisplayDepthMap();

    /**
     * @brief mergeOptimizationOffset
     *
     * Merges the current keyframe optimization offset to all working entities.
     */
    void mergeOptimizationOffset();

    void addTimingSamples();
    void takeRelocalizeResult();

private:

    /// EXCLUSIVELY MAPPING THREAD (+ init).
    DepthMap*               map;
    TrackingReference*      mappingTrackingReference;

    /// During re-localization used.
    std::vector<Frame*> 	KFForReloc;
    int 					nextRelocIdx;
    std::shared_ptr<Frame> 	latestFrameTriedForReloc;

    /// Handle of mapping thread.
    boost::thread   thread_mapping;

    /// Flag of necessity for Depth map output.
    bool            depthMapScreenshotFlag;

    /// Name for a file for output.
    std::string     depthMapScreenshotFilename;

    /// USED DURING RE-LOCALIZATION ONLY.
    Relocalizer relocalizer;

//******************************************************************************
// ******************** Работа с Optimization **********************************


public:

    /// Time of iteration in milliseconds.
    float   msOptimizationIteration;

    /// Amount of iterations.
    int     nOptimizationIteration;

    // ????
    float   nAvgOptimizationIteration;

    /// Flag of optimization.
    bool 	doFinalOptimization;

public:
    // Функция оптимизации
//    bool optimizationIteration(int itsPerTry, float minChange);

private:
    boost::thread thread_optimization;

private:

    /**
     * @brief optimizationThreadLoop
     *
     * Function of optimization thread.
     */
    void optimizationThreadLoop();

//******************************************************************************
// ********************* Работа с Сonatraint ***********************************

public:
    float   msFindConstraintsItaration;
    int     nFindConstraintsItaration;
    float   nAvgFindConstraintsItaration;

public:
    // Проверить, не перенсти ли ее к кадрам
//    int findConstraintsForNewKeyFrames( Frame*  newKeyFrame                 ,
//                                        bool    forceParent         = true  ,
//                                        bool    useFABMAP           = true  ,
//                                        float   closeCandidatesTH   = 1.0       );

private:

    // === EXCLUSIVELY FIND-CONSTRAINT THREAD (+ init) ===
    TrackableKeyFrameSearch* 	trackableKeyFrameSearch;

    Sim3Tracker* 				constraintTracker;
    SE3Tracker* 				constraintSE3Tracker;

    TrackingReference* 			newKFTrackingReference;
    TrackingReference*			candidateTrackingReference;

    boost::thread               thread_constraint_search;

    int                         lastNumConstraintsAddedOnFullRetrack;

private:

    /**
     * @brief constraintSearchThreadLoop
     *
     * Function of a thread.
     */
    void constraintSearchThreadLoop();

    // Я коментил
//    void testConstraint(    Frame*              candidate                       ,
//                            KFConstraintStruct* &e1_out                         ,
//                            KFConstraintStruct* &e2_out                         ,
//                            Sim3                candidateToFrame_initialEstimate,
//                            float               strictness                        );

//    /** Calculates a scale independent error norm for reciprocal tracking results
//     *  a and b with associated information matrices.
//    */
    // Тоже я
//    float tryTrackSim3( TrackingReference* A        ,
//                        TrackingReference* B        ,
//                        int lvlStart                ,
//                        int lvlEnd                  ,
//                        bool useSSE                 ,
//                        Sim3 &AtoB                  ,
//                        Sim3 &BtoA                  ,
//                        KFConstraintStruct* e1=0    ,
//                        KFConstraintStruct* e2=0        );




    //*********************************************
public:
    float   msTrackFrame;
    float   msFindReferences;

    int     nTrackFrame;
    int     nFindReferences;

    float   nAvgTrackFrame;
    float   nAvgFindReferences;

    /// Control of frequency update
    struct timeval lastHzUpdate;

private:

    // ============= EXCLUSIVELY TRACKING THREAD (+ init) ===============
    ///  Tracking reference for current keyframe. Only used by tracking.
    TrackingReference*	m_pCurrentFrameTrackingReference;
    SE3Tracker* 		tracker;

    // ============= SHARED ENTITIES ============= */
    float tracking_lastResidual;
    float tracking_lastUsage;
	float tracking_lastGoodPerBad;
    float tracking_lastGoodPerTotal;

    float lastTrackingClosenessScore;

    /// For sequential operation. Set in Mapping, read in Tracking.
	boost::condition_variable  	newFrameMappedSignal;
	boost::mutex 				newFrameMappedMutex;

    /// Has own locks
    KeyFrameGraph* 		keyFrameGraph;

    /**
     *  Tracking:    if (!create) set candidate, set     create.
     *  Mapping:     if (create ) use candidate, reset   create.
     *  => no locking required.
     */
    std::shared_ptr<Frame>      latestTrackedFrame;
    bool                        createNewKeyFrame;

    /// PUSHED by Mapping, READ & CLEARED by constraintFinder
    std::deque< Frame* >        newKeyFrames;
    boost::mutex                newKeyFrameMutex;
    boost::condition_variable   newKeyFrameCreatedSignal;

    /// Optimization thread
    bool                        newConstraintAdded;
    boost::mutex                newConstraintMutex;
    boost::condition_variable   newConstraintCreatedSignal;
    boost::mutex                g2oGraphAccessMutex;

    /// optimization merging. SET in Optimization, merged in Mapping.
	bool haveUnmergedOptimizationOffset;


    /// Mutex to lock frame pose consistency. within a shared lock of this, *->getScaledCamToWorld() is
    /// GUARANTEED to give the same result each call, and to be compatible to each other.
    /// locked exclusively during the pose-update by Mapping.
    boost::shared_mutex poseConsistencyMutex;

    //************************ Непонятно что ***************************
public:

    /// Settings. Constant from construction onward.
    int width;

    /// Settings. Constant from construction onward.
    int height;

    Eigen::Matrix3f K;

    const bool  SLAMEnabled;

    bool        trackingIsGood;

public:
//    /** Does an offline optimization step. */
    // Почиму-то нигде не используется ( может быть для обработки изиображений ?? )
//    void optimizeGraph();

private:
    // Функция устанавливает флаг запроса карты глубин и имя фаила для вывода
    // Нигде не вызывается
    void requestDepthMapScreenshot( const std::string& filename );

//    /** Returns the current pose estimate. */
//    SE3 getCurrentPoseEstimate();
//    std::vector<FramePoseStruct*, Eigen::aligned_allocator<lsd_slam::FramePoseStruct*> > getAllPoses();

    /// It was in settings.h
    bool displayDepthMap;

    /// TODO: Bring OUT from this Classes
    DebugImage* m_pDebugImage;
    // Individual / no locking
    /// No lock required
    lsdSlamOutput*	m_pOutputWrapper;
};

}
