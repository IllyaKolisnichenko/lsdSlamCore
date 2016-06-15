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

#include "SlamSystem.h"

#include "DataStructures/Frame.h"
//#include "DataStructures/FrameMemory.h"

#include "DepthEstimation/DepthMap.h"

#include "Sim3Tracker.h"
#include "SE3Tracker.h"
#include "TrackingReference.h"

#include "globalFuncs.h"

#include "KeyFrameGraph.h"

#include "TrackableKeyFrameSearch.h"
//#include "g2oTypeSim3Sophus.h"

#include "ImageDisplay.h"
#include "Output3DWrapper/myoutput3dwrapper.h"

//#include <g2o/core/robust_kernel_impl.h>

#include "deque"

// for mkdir
#include <sys/types.h>
#include <sys/stat.h>

#ifdef ANDROID
#include <android/log.h>
#endif

#include "opencv2/opencv.hpp"

using namespace lsd_slam;

// Constructor
SlamSystem::SlamSystem( int w, int h, Eigen::Matrix3f K, bool enableSLAM ) :
    SLAMEnabled		( enableSLAM	),
    displayDepthMap (   true        ),
    relocalizer     ( w,h,K			)
{
    // The sides of an image must be divisible by 16
	if(w%16 != 0 || h%16!=0)
	{
        // Output the image
		printf("image dimensions must be multiples of 16! Please crop your images / video accordingly.\n");
        // Stop processing
		assert(false);
	}

    // Set the parameters of the camera
	this->width 	= w;
	this->height 	= h;
	this->K 		= K;

    // Tracking is good by default
	trackingIsGood 	= true;

	currentKeyFrame 				= nullptr;
	trackingReferenceFrameSharedPT	= nullptr;

    // Craete a new graph
    keyFrameGraph 					= new KeyFrameGraph();

	createNewKeyFrame 				= false;

    // Create an instance of the depth map
    map =  new DepthMap( this->width,
                         this->height,
                         this->K        );

	newConstraintAdded 				= false;
	haveUnmergedOptimizationOffset 	= false;

    // Create an instance of the tracker
    tracker = new SE3Tracker( this->width,
                              this->height,
                              this->K       );

	// Do not use more than 4 levels for odometry tracking
    for (int level = 4; level < PYRAMID_LEVELS; ++level)
        tracker->settings.maxItsPerLvl[level] = 0;

    trackingReference 			= new TrackingReference();
    mappingTrackingReference 	= new TrackingReference();

    if( SLAMEnabled )
    {
        trackableKeyFrameSearch 	= new TrackableKeyFrameSearch( keyFrameGraph,
                                                                   this->width,
                                                                   this->height,
                                                                   this->K);

        constraintTracker 			= new Sim3Tracker   (   this->width,
                                                            this->height,
                                                            this->K         );

        constraintSE3Tracker 		= new SE3Tracker    (   this->width,
                                                            this->height,
                                                            this->K         );

        newKFTrackingReference 		= new TrackingReference();
        candidateTrackingReference	= new TrackingReference();
    }
    else
    {
        trackableKeyFrameSearch 	= 0;
        constraintTracker 			= 0;

        constraintSE3Tracker 		= 0;

        newKFTrackingReference 		= 0;
        candidateTrackingReference	= 0;
    }

    outputWrapper = 0;

	keepRunning 				= true;
	doFinalOptimization 		= false;
	depthMapScreenshotFlag 		= false;
	lastTrackingClosenessScore 	= 0;

    // Run the mapping function in a new Thread
    // thread_mapping = boost::thread(&SlamSystem::mappingThreadLoop, this);

    if( SLAMEnabled )
	{
        // The thread of renewal of constraints
//      thread_constraint_search    = boost::thread(&SlamSystem::constraintSearchThreadLoop,    this);
        // The thread of optimization
        //      thread_optimization         = boost::thread(&SlamSystem::optimizationThreadLoop,        this);
	}

    // Additional threads run only if SLAM is turned ON
	msTrackFrame 				= 0;
	msOptimizationIteration 	= 0;
	msFindConstraintsItaration	= 0;
	msFindReferences 			= 0;

    // mapping ThreadLoop run in any case
	nTrackFrame 				= 0;
	nOptimizationIteration		= 0;
	nFindConstraintsItaration	= 0;
	nFindReferences 			= 0;
	nAvgTrackFrame 				= 0;

	nAvgOptimizationIteration 		= 0;
	nAvgFindConstraintsItaration	= 0;

	nAvgFindReferences	= 0;

	gettimeofday(&lastHzUpdate, NULL);
}

SlamSystem::~SlamSystem()
{
	keepRunning = false;

    // Make sure none is waiting for something.
	printf("... waiting for SlamSystem's threads to exit\n");

	newFrameMappedSignal.notify_all();
	unmappedTrackedFramesSignal.notify_all();
	newKeyFrameCreatedSignal.notify_all();
	newConstraintCreatedSignal.notify_all();

	thread_mapping.join();
	thread_constraint_search.join();
	thread_optimization.join();
	printf("DONE waiting for SlamSystem's threads to exit\n");

    if(trackableKeyFrameSearch != 0)
        delete trackableKeyFrameSearch;

    if(constraintTracker != 0)
        delete constraintTracker;

    if(constraintSE3Tracker != 0)
        delete constraintSE3Tracker;

    if(newKFTrackingReference != 0)
        delete newKFTrackingReference;

    if(candidateTrackingReference != 0)
        delete candidateTrackingReference;

    delete mappingTrackingReference;
    delete map;
    delete trackingReference;
    delete tracker;

    // Make shure to reset all shared pointers to all frames before deleting the keyframegraph!
	unmappedTrackedFrames.clear();
	latestFrameTriedForReloc.reset();
	latestTrackedFrame.reset();
	currentKeyFrame.reset();
	trackingReferenceFrameSharedPT.reset();

    // Delete keyframe graph
    delete keyFrameGraph;

	FrameMemory::getInstance().releaseBuffes();

    Util::closeAllWindows();
}

void SlamSystem::setVisualization(Output3DWrapper* outputWrapper)
{
    this->outputWrapper = outputWrapper;
}

void SlamSystem::mergeOptimizationOffset()
{
    // Update all vertices that are in the graph!
	poseConsistencyMutex.lock();

    // Do we need to publish(visualize) ???
	bool needPublish = false;
	if(haveUnmergedOptimizationOffset)
    {
        keyFrameGraph->keyframesAllMutex.lock_shared();
        for(unsigned int i=0;i<keyFrameGraph->keyframesAll.size(); i++)
            keyFrameGraph->keyframesAll[i]->pose->applyPoseGraphOptResult();
        keyFrameGraph->keyframesAllMutex.unlock_shared();

		haveUnmergedOptimizationOffset = false;
		needPublish = true;
	}

	poseConsistencyMutex.unlock();

    if(needPublish)
        publishKeyframeGraph();
}

void SlamSystem::mappingThreadLoop() // Works independent from tracking
{
    // Notify about launching of the thread
    printf("Started mapping thread!\n");

    // Until the flag of launching is TRUE
    // Flag is set to FALSE only in destructor for stopping the threads
    while(keepRunning)
    {
        // Do the next iteration
        bool mappingResult = doMappingIteration();
        // If it was unsuccessful
        if (!mappingResult)
        {
            // Lock the mutex
            boost::unique_lock<boost::mutex> lock( unmappedTrackedFramesMutex );

            // Waiting for a message from the mutex
            unmappedTrackedFramesSignal.timed_wait( lock,
                                                    boost::posix_time::milliseconds(200)    );	// Slight chance of deadlock otherwise
            // Unlock the mutex
            lock.unlock();
        }

        // Notify trackFrame that iteration is finished
        newFrameMappedMutex.lock();
        // Notify that iteration of mapping is finished
        newFrameMappedSignal.notify_all();
        newFrameMappedMutex.unlock();
    }

    // Quit from the mapping loop
    printf("Exited mapping thread \n");
}

// Проверил - можно просто расскоментировать
//void SlamSystem::finalize()
//{
//	printf("Finalizing Graph... finding final constraints!!\n");

//	lastNumConstraintsAddedOnFullRetrack = 1;

//	while(lastNumConstraintsAddedOnFullRetrack != 0)
//	{
//		doFullReConstraintTrack = true;
//		usleep(200000);
//	}

//	printf("Finalizing Graph... optimizing!!\n");
//	doFinalOptimization = true;
//	newConstraintMutex.lock();
//	newConstraintAdded = true;
//	newConstraintCreatedSignal.notify_all();
//	newConstraintMutex.unlock();
//	while(doFinalOptimization)
//	{
//		usleep(200000);
//	}

//	printf("Finalizing Graph... publishing!!\n");
//	unmappedTrackedFramesMutex.lock();
//	unmappedTrackedFramesSignal.notify_one();
//	unmappedTrackedFramesMutex.unlock();
//	while(doFinalOptimization)
//	{
//		usleep(200000);
//	}
//	boost::unique_lock<boost::mutex> lock(newFrameMappedMutex);
//	newFrameMappedSignal.wait(lock);
//	newFrameMappedSignal.wait(lock);

//	usleep(200000);

//	printf("Done Finalizing Graph.!!\n");
//}

// Проверил - можно просто расскоментировать
//void SlamSystem::constraintSearchThreadLoop() // Changing of the constraints
//{
//	printf("Started  constraint search thread!\n");

//	boost::unique_lock<boost::mutex> lock(newKeyFrameMutex); // Declare the buffer of new "unprocessed" KF's

//	int failedToRetrack = 0;    // Counter

//  while(keepRunning)          // While working
//	{
//		if(newKeyFrames.size() == 0)    // If the buffer is empty
//		{
//			lock.unlock();              // Unlock

//            keyFrameGraph->keyframesForRetrackMutex.lock();   // Block the data

//            bool doneSomething = false;

//            if(keyFrameGraph->keyframesForRetrack.size() > 10) // For a buffer that is more than 10
//            {
//                std::deque< Frame* >::iterator toReTrack = keyFrameGraph->keyframesForRetrack.begin() + (rand() % (keyFrameGraph->keyframesForRetrack.size()/3));
//                Frame* toReTrackFrame = *toReTrack;   // Random frame 1 of 10

//                keyFrameGraph->keyframesForRetrack.erase(toReTrack);
//                keyFrameGraph->keyframesForRetrack.push_back(toReTrackFrame);

//                keyFrameGraph->keyframesForRetrackMutex.unlock();

//                int found = findConstraintsForNewKeyFrames(toReTrackFrame, false, false, 2.0);
//                if(found == 0)    // Nothing is found
//                    failedToRetrack++;    // Increment the counter
//                else
//                    failedToRetrack=0;    // Null the counter

//                if(failedToRetrack < (int)keyFrameGraph->keyframesForRetrack.size() - 5)
//                    doneSomething = true;
//            }
//            else
//                keyFrameGraph->keyframesForRetrackMutex.unlock(); // Unlock the data

//            lock.lock();

//            if(!doneSomething)    // If nothing to do
//            {
//                if(enablePrintDebugInfo && printConstraintSearchInfo)
//                    printf("nothing to re-track... waiting.\n");
//                newKeyFrameCreatedSignal.timed_wait(lock,boost::posix_time::milliseconds(500)); // Output the debugging information and put to sleep the tread for 500ms

//            }
//        }
//		else
//		{
//          // Receive the first frame
//			Frame* newKF = newKeyFrames.front();`
//			newKeyFrames.pop_front();
//			lock.unlock();

//          // Counting time
//			struct timeval tv_start, tv_end;
//			gettimeofday(&tv_start, NULL);

//			findConstraintsForNewKeyFrames(newKF, true, true, 1.0);
//			failedToRetrack=0;
//			gettimeofday(&tv_end, NULL);

//          // Time for searching the constraints
//			msFindConstraintsItaration = 0.9*msFindConstraintsItaration + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
//			nFindConstraintsItaration++;

//			FrameMemory::getInstance().pruneActiveFrames();
//			lock.lock();
//		}

//		if(doFullReConstraintTrack)
//		{
//			lock.unlock();
//			printf("Optizing Full Map!\n");

//            int added = 0;
//            for(unsigned int i=0;i<keyFrameGraph->keyframesAll.size();i++)
//            {
//                if(keyFrameGraph->keyframesAll[i]->pose->isInGraph)
//                    added += findConstraintsForNewKeyFrames(keyFrameGraph->keyframesAll[i], false, false, 1.0); // Page 25
//            }

//			printf("Done optizing Full Map! Added %d constraints.\n", added);
//          // Who is setting it to TRUE ???
//			doFullReConstraintTrack = false;

//			lastNumConstraintsAddedOnFullRetrack = added;
//			lock.lock();
//		}
//	}

//	printf("Exited constraint search thread \n");
//}

// The thread of optimization
void SlamSystem::optimizationThreadLoop()
{
    // Notify that thread is launched
	printf("Started optimization thread \n");

    // While working
    while( keepRunning )
	{
        // Catching the thread
		boost::unique_lock<boost::mutex> lock(newConstraintMutex);

        // If new constraints aren't added
        if( !newConstraintAdded )
            // Slight chance of deadlock otherwise
            // Check if new constraints are added
            newConstraintCreatedSignal.timed_wait( lock, boost::posix_time::milliseconds(2000) );

        // If new constraints are added than notify that processed
        newConstraintAdded = false;

        // Unlock the thread
		lock.unlock();

        // Doing the final optimization
        // To do: find a place where flag is set to TRUE
        if( doFinalOptimization )
		{
            // Notify about this
			printf("doing final optimization iteration!\n");

            // Directly optimization
            // optimizationIteration(50, 0.001);

            // Set the flag to FALSE
			doFinalOptimization = false;
		}

        // High accuracy optimization
        // while( optimizationIteration(5, 0.02) );
	}

	printf("Exited optimization thread \n");
}

void SlamSystem::publishKeyframeGraph()
{
    if (outputWrapper != nullptr)
        outputWrapper->publishKeyframeGraph(keyFrameGraph);
}

void SlamSystem::requestDepthMapScreenshot(const std::string& filename)
{
    depthMapScreenshotFilename  = filename;
    depthMapScreenshotFlag      = true;
}

void SlamSystem::finishCurrentKeyframe()
{
    if(enablePrintDebugInfo && printThreadingInfo)
        printf("FINALIZING KF %d\n", currentKeyFrame->id());

    map->finalizeKeyFrame();

    if(SLAMEnabled)
    {
        mappingTrackingReference->importFrame(currentKeyFrame.get());
        currentKeyFrame->setPermaRef(mappingTrackingReference);
        mappingTrackingReference->invalidate();

        if(currentKeyFrame->idxInKeyframes < 0)
        {
            keyFrameGraph->keyframesAllMutex.lock();
            currentKeyFrame->idxInKeyframes = keyFrameGraph->keyframesAll.size();

            keyFrameGraph->keyframesAll.push_back(currentKeyFrame.get());

            keyFrameGraph->totalPoints += currentKeyFrame->numPoints;
            keyFrameGraph->totalVertices ++;
            keyFrameGraph->keyframesAllMutex.unlock();

            newKeyFrameMutex.lock();
            newKeyFrames.push_back(currentKeyFrame.get());
            newKeyFrameCreatedSignal.notify_all();
            newKeyFrameMutex.unlock();
        }
    }

    if(outputWrapper != 0 )
        outputWrapper->publishKeyframe( currentKeyFrame.get() );
}

void SlamSystem::discardCurrentKeyframe()
{
    if(enablePrintDebugInfo && printThreadingInfo)
        printf("DISCARDING KF %d\n", currentKeyFrame->id());

    if(currentKeyFrame->idxInKeyframes >= 0)
    {
        printf("WARNING: trying to discard a KF that has already been added to the graph... finalizing instead.\n");
        finishCurrentKeyframe();
        return;
    }

    map->invalidate();

    keyFrameGraph->allFramePosesMutex.lock();
    for(FramePoseStruct* p : keyFrameGraph->allFramePoses)
    {
        if(p->trackingParent != 0 && p->trackingParent->frameID == currentKeyFrame->id())
            p->trackingParent = 0;
    }
    keyFrameGraph->allFramePosesMutex.unlock();

    keyFrameGraph->idToKeyFrameMutex.lock();
    keyFrameGraph->idToKeyFrame.erase(currentKeyFrame->id());
    keyFrameGraph->idToKeyFrameMutex.unlock();
}

void SlamSystem::createNewCurrentKeyframe(std::shared_ptr<Frame> newKeyframeCandidate)
{
    if(enablePrintDebugInfo && printThreadingInfo)
        printf("CREATE NEW KF %d from %d\n", newKeyframeCandidate->id(), currentKeyFrame->id());


    if(SLAMEnabled)
    {
        // add NEW keyframe to id-lookup
        keyFrameGraph->idToKeyFrameMutex.lock();
        keyFrameGraph->idToKeyFrame.insert(std::make_pair(newKeyframeCandidate->id(), newKeyframeCandidate));
        keyFrameGraph->idToKeyFrameMutex.unlock();
    }

    // propagate & make new.
    map->createKeyFrame(newKeyframeCandidate.get());

    if(printPropagationStatistics)
    {

        Eigen::Matrix<float, 20, 1> data;
        data.setZero();
        data[0] = runningStats.num_prop_attempts / ((float)width*height);
        data[1] = (runningStats.num_prop_created + runningStats.num_prop_merged) / (float)runningStats.num_prop_attempts;
        data[2] = runningStats.num_prop_removed_colorDiff / (float)runningStats.num_prop_attempts;

        outputWrapper->publishDebugInfo(data);
    }

    currentKeyFrameMutex.lock();
    currentKeyFrame = newKeyframeCandidate;
    currentKeyFrameMutex.unlock();
}

void SlamSystem::loadNewCurrentKeyframe(Frame* keyframeToLoad)
{
    if(enablePrintDebugInfo && printThreadingInfo)
        printf("RE-ACTIVATE KF %d\n", keyframeToLoad->id());

    map->setFromExistingKF(keyframeToLoad);

    if(enablePrintDebugInfo && printRegularizeStatistics)
        printf("re-activate frame %d!\n", keyframeToLoad->id());

    currentKeyFrameMutex.lock();

    currentKeyFrame = keyFrameGraph->idToKeyFrame.find(keyframeToLoad->id())->second;
    currentKeyFrame->depthHasBeenUpdatedFlag = false;
    currentKeyFrameMutex.unlock();
}

void SlamSystem::changeKeyframe(bool noCreate, bool force, float maxScore)
{
    Frame* newReferenceKF = 0;

    std::shared_ptr<Frame> newKeyframeCandidate = latestTrackedFrame;

    if(doKFReActivation && SLAMEnabled)
    {
        struct timeval tv_start, tv_end;
        gettimeofday(&tv_start, NULL);

        newReferenceKF = trackableKeyFrameSearch->findRePositionCandidate( newKeyframeCandidate.get(),
                                                                           maxScore                     );

        gettimeofday( &tv_end, NULL );
        msFindReferences = 0.9*msFindReferences + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
        nFindReferences++;
    }

    if(newReferenceKF != 0)
        loadNewCurrentKeyframe(newReferenceKF);
    else
    {
        if(force)
        {
            if(noCreate)
            {
                trackingIsGood = false;
                nextRelocIdx = -1;
                printf("mapping is disabled & moved outside of known map. Starting Relocalizer!\n");
            }
            else
                createNewCurrentKeyframe(newKeyframeCandidate);
        }
    }

    createNewKeyFrame = false;
}

bool SlamSystem::updateKeyframe()
{

    // Make new list of shared ptr
    std::deque< std::shared_ptr<Frame> > references;

    // Block list (block an access to the buffer of frames)
    unmappedTrackedFramesMutex.lock();

    // Remove frames that have a different tracking parent
    // Until the queue is not empty (more than 0)
    while(       unmappedTrackedFrames.size() > 0                   &&
                // If first element doesn't have a parent (wasn't processed by SE3Tracker::TrackFrame)
            (   !unmappedTrackedFrames.front()->hasTrackingParent() ||
                // or have different parent KeyFrame
                 unmappedTrackedFrames.front()->getTrackingParent() != currentKeyFrame.get() )   )
    {
        // Delete data (clearing the memory)
        unmappedTrackedFrames.front()->clear_refPixelWasGood();
        // Delete element from list
        unmappedTrackedFrames.pop_front();
    }

    // Clone the list
    // If list is still not empty
    if( unmappedTrackedFrames.size() > 0 )
    {
        // Copy the list
        // For all the elements in the list
        for(unsigned int i = 0; i < unmappedTrackedFrames.size(); i++)
            // Copy the element
            references.push_back( unmappedTrackedFrames[i] );

        // Receive pointer to the first element
        std::shared_ptr<Frame> popped = unmappedTrackedFrames.front();

        // Delete first element from the list
        unmappedTrackedFrames.pop_front();

        // Unblock the list
        unmappedTrackedFramesMutex.unlock();

        // Print debug info
        if(enablePrintDebugInfo && printThreadingInfo)
            printf( "MAPPING %d on %d to %d (%d frames)\n",
                    currentKeyFrame->id(),
                    references.front()->id(),
                    references.back()->id(),
                    (int)references.size()      );

        // Update selected Key Frame
        map->updateKeyframe( references );

        // Clear some data ???
        popped->clear_refPixelWasGood();

        // Clear temporary list
        references.clear();
    }
    else
    {
        // Unblock list
        unmappedTrackedFramesMutex.unlock();
        // Return
        return false;
    }

    // Publish debugging information
    if( enablePrintDebugInfo        &&
        printRegularizeStatistics       )
    {
        Eigen::Matrix<float, 20, 1> data;
        data.setZero();
        data[0] = runningStats.num_reg_created;
        data[2] = runningStats.num_reg_smeared;
        data[3] = runningStats.num_reg_deleted_secondary;
        data[4] = runningStats.num_reg_deleted_occluded;
        data[5] = runningStats.num_reg_blacklisted;

        data[6] = runningStats.num_observe_created;
        data[7] = runningStats.num_observe_create_attempted;
        data[8] = runningStats.num_observe_updated;
        data[9] = runningStats.num_observe_update_attempted;

        data[10] = runningStats.num_observe_good;
        data[11] = runningStats.num_observe_inconsistent;
        data[12] = runningStats.num_observe_notfound;
        data[13] = runningStats.num_observe_skip_oob;
        data[14] = runningStats.num_observe_skip_fail;

        outputWrapper->publishDebugInfo(data);
    }

    // If everything is OK
    if( outputWrapper != 0  &&
        continuousPCOutput  &&
        currentKeyFrame != 0        )
        // Publish current Key Frame for redraw Point Cloud
        outputWrapper->publishKeyframe( currentKeyFrame.get() );

    return true;
}

void SlamSystem::addTimingSamples()
{
    // Add time mark
    map->addTimingSample();

    struct timeval now;

    gettimeofday(&now, NULL);

    float sPassed = ((now.tv_sec-lastHzUpdate.tv_sec) + (now.tv_usec-lastHzUpdate.tv_usec)/1000000.0f);

    if( sPassed > 1.0f )
    {
        nAvgTrackFrame              =   0.8 * nAvgTrackFrame +
                                        0.2 * (nTrackFrame / sPassed);
        nTrackFrame = 0;

        nAvgOptimizationIteration   =   0.8 * nAvgOptimizationIteration +
                                        0.2 * (nOptimizationIteration / sPassed);
        nOptimizationIteration      =   0;

        nAvgFindReferences          =   0.8 * nAvgFindReferences +
                                        0.2 * (nFindReferences / sPassed);
        nFindReferences             =   0;

        if(trackableKeyFrameSearch != 0)
        {
            trackableKeyFrameSearch->nAvgTrackPermaRef  =   0.8 * trackableKeyFrameSearch->nAvgTrackPermaRef +
                                                            0.2 * (trackableKeyFrameSearch->nTrackPermaRef / sPassed    );
            trackableKeyFrameSearch->nTrackPermaRef     =   0;
        }

        nAvgFindConstraintsItaration =  0.8 * nAvgFindConstraintsItaration +
                                        0.2 * (nFindConstraintsItaration / sPassed);

        nFindConstraintsItaration    =  0;

        nAvgOptimizationIteration    =  0.8 * nAvgOptimizationIteration +
                                        0.2 * (nOptimizationIteration / sPassed);

        nOptimizationIteration = 0;

        lastHzUpdate = now;

        // If output of the debugging information is turned ON
        if(enablePrintDebugInfo && printOverallTiming)
        {
            // Output of the debugging information
            printf("MapIt: %3.1fms (%.1fHz); Track: %3.1fms (%.1fHz); Create: %3.1fms (%.1fHz); FindRef: %3.1fms (%.1fHz); PermaTrk: %3.1fms (%.1fHz); Opt: %3.1fms (%.1fHz); FindConst: %3.1fms (%.1fHz);\n",
                    map->msUpdate,
                    map->nAvgUpdate,
                    msTrackFrame,
                    nAvgTrackFrame,
                    map->msCreate + map->msFinalize,
                    map->nAvgCreate,
                    msFindReferences,
                    nAvgFindReferences,
                    trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->msTrackPermaRef     : 0,
                    trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->nAvgTrackPermaRef   : 0,
                    msOptimizationIteration,
                    nAvgOptimizationIteration,
                    msFindConstraintsItaration,
                    nAvgFindConstraintsItaration        );
        }
    }
}

void SlamSystem::debugDisplayDepthMap()
{
    // Prepare data for visualization
    map->debugPlotDepthMap();

    double scale = 1;
    // if(currentKeyFrame != 0 && currentKeyFrame != 0) // ??????????
    if( currentKeyFrame != 0 )
         scale = currentKeyFrame->getScaledCamToWorld().scale();

    // Debug plot depthmap
    char buf1[200];
    char buf2[200];

    snprintf(   buf1,
                200,
                "Map: Upd %3.0fms (%2.0fHz); Trk %3.0fms (%2.0fHz); %d / %d / %d",
                map->msUpdate,
                map->nAvgUpdate,
                msTrackFrame,
                nAvgTrackFrame,
                currentKeyFrame->numFramesTrackedOnThis,
                currentKeyFrame->numMappedOnThis,
                (int)unmappedTrackedFrames.size()           );

    snprintf(   buf2,
                200,
                "dens %2.0f%%; good %2.0f%%; scale %2.2f; res %2.1f/; usg %2.0f%%; Map: %d F, %d KF, %d E, %.1fm Pts",
                100*currentKeyFrame->numPoints/(float)(width*height),
                100*tracking_lastGoodPerBad,
                scale,
                tracking_lastResidual,
                100*tracking_lastUsage,
                (int)keyFrameGraph->allFramePoses.size(),
                keyFrameGraph->totalVertices,
                (int)keyFrameGraph->edgesAll.size(),
                1e-6 * (float)keyFrameGraph->totalPoints);

    if(onSceenInfoDisplay)
        printMessageOnCVImage( map->debugImageDepth, buf1, buf2 );

    if (displayDepthMap)
        Util::displayImage( "DebugWindow DEPTH", map->debugImageDepth, false );

    int pressedKey = Util::waitKey(1);
    handleKey(pressedKey);
}

void SlamSystem::takeRelocalizeResult()
{
    Frame*                  keyframe;
    int                     succFrameID;
    SE3                     succFrameToKF_init;
    std::shared_ptr<Frame>  succFrame;

    relocalizer.stop();
    relocalizer.getResult(keyframe, succFrame, succFrameID, succFrameToKF_init);
    assert(keyframe != 0);

    loadNewCurrentKeyframe(keyframe);

    currentKeyFrameMutex.lock();
    trackingReference->importFrame(currentKeyFrame.get());
    trackingReferenceFrameSharedPT = currentKeyFrame;
    currentKeyFrameMutex.unlock();

    tracker->trackFrame(    trackingReference   ,
                            succFrame.get()     ,
                            succFrameToKF_init      );

    if(     !tracker->trackingWasGood ||
            tracker->lastGoodCount / (tracker->lastGoodCount + tracker->lastBadCount) < 1-0.75f*(1-MIN_GOODPERGOODBAD_PIXEL))
    {
        if(enablePrintDebugInfo && printRelocalizationInfo)
            printf("RELOCALIZATION FAILED BADLY! discarding result.\n");
        trackingReference->invalidate();
    }
    else
    {
        keyFrameGraph->addFrame(succFrame.get());

        unmappedTrackedFramesMutex.lock();

        if(unmappedTrackedFrames.size() < 50)
            unmappedTrackedFrames.push_back(succFrame);

        unmappedTrackedFramesMutex.unlock();

        currentKeyFrameMutex.lock();

        createNewKeyFrame   = false;
        trackingIsGood      = true;

        currentKeyFrameMutex.unlock();
    }
}

// Called in a function of the appropriate thread
bool SlamSystem::doMappingIteration()
{
    // If current frame is not set
    if(currentKeyFrame == 0)
        // Return error
        return false;

    // If no need to do mapping and parameter is still not changed
    if( !doMapping && currentKeyFrame->idxInKeyframes < 0 )
    {
        // If the amount of frames processed in this selection is MORE or EQUAL to given
        if( currentKeyFrame->numMappedOnThisTotal >= MIN_NUM_MAPPED )
            // Finish the Key Frame
            finishCurrentKeyframe();
        else
            // Discard the Key Frame
            discardCurrentKeyframe();

        // Something to discard
        map->invalidate();

        // Сообщить об окончании формирования ключевого кадра с номером
        printf( "Finished KF %d as Mapping got disabled!\n", currentKeyFrame->id() );

        // Нужно вникнуть...
        changeKeyframe(true, true, 1.0f);
    }

    // Need to update changes after iteration of thread optimization is done
    mergeOptimizationOffset();

    // For debugging information
    addTimingSamples();

    if( dumpMap )
    {
        keyFrameGraph->dumpMap(packagePath+"/save");
        dumpMap = false;
    }

    // Set mappingFrame
    if(trackingIsGood) // Find the reasons of changing
    {
        if(!doMapping) // If don't need to do
        {
            //printf("tryToChange refframe, lastScore %f!\n", lastTrackingClosenessScore);
            if(lastTrackingClosenessScore > 1)
                changeKeyframe(true, false, lastTrackingClosenessScore * 0.75);

            if (displayDepthMap || depthMapScreenshotFlag)
                debugDisplayDepthMap();

            return false;
        }

        // If tracking is OK and doMapping is set to FALSE next part isn't performed
        if (createNewKeyFrame)
        {
            finishCurrentKeyframe();
            changeKeyframe(false, true, 1.0f);

            if ( displayDepthMap        ||
                 depthMapScreenshotFlag     )
                debugDisplayDepthMap();
        }
        else
        {
            bool didSomething = updateKeyframe();

            if ( displayDepthMap         ||
                 depthMapScreenshotFlag     )
                debugDisplayDepthMap();
            if(!didSomething)
                return false;
        }

        return true;
    }
    else
    {
        // Invalidate the map if it was valid
        if(map->isValid()) // Depth map
        {
            if(currentKeyFrame->numMappedOnThisTotal >= MIN_NUM_MAPPED)
                finishCurrentKeyframe();
            else
                discardCurrentKeyframe();

            map->invalidate();
        }

        // Start relocalizer if it isn't already running
        if(!relocalizer.isRunning)
            relocalizer.start(keyFrameGraph->keyframesAll);

        // Did we find a frame to relocalize with?
        if( relocalizer.waitResult(50) )
            takeRelocalizeResult();

        return true;
    }
}

// Проверил - можно просто расскоментировать
//void SlamSystem::gtDepthInit(uchar* image, float* depth, double timeStamp, int id)
//{
//	printf("Doing GT initialization!\n");

//	currentKeyFrameMutex.lock();

//	currentKeyFrame.reset(new Frame(id, width, height, K, timeStamp, image));
//	currentKeyFrame->setDepthFromGroundTruth(depth);

//	map->initializeFromGTDepth(currentKeyFrame.get());

//	keyFrameGraph->addFrame(currentKeyFrame.get());

//	currentKeyFrameMutex.unlock();

//	if(doSlam)
//	{
//		keyFrameGraph->idToKeyFrameMutex.lock();
//		keyFrameGraph->idToKeyFrame.insert(std::make_pair(currentKeyFrame->id(), currentKeyFrame));
//		keyFrameGraph->idToKeyFrameMutex.unlock();
//	}

//	if(continuousPCOutput && outputWrapper != 0) outputWrapper->publishKeyframe(currentKeyFrame.get());

//	printf("Done GT initialization!\n");
//}

// Initialization of the first Key Frame
void SlamSystem::randomInit(uchar* image, double timeStamp, int id)
{
    // Notify that initialization is started
    printf("Doing Random initialization!\n");

    // If the parameter is OFF
    if( !doMapping )
        // Notify about it
        printf("WARNING: mapping is disabled, but we just initialized... THIS WILL NOT WORK! Set doMapping to true.\n");

    // Lock an access to the current Key Frame
    currentKeyFrameMutex.lock();

    // Create a new frame and reset ( "reset" will replace existing pointer)
    currentKeyFrame.reset( new Frame( id, width, height, K, timeStamp, image ) );

    // Initialize the depth map
    map->initializeRandomly( currentKeyFrame.get() );

    // Add the Key Frame to the graph
    // Save the position of the frame "pose" in the list
    keyFrameGraph->addFrame( currentKeyFrame.get() );

    // Unlock the thread
    currentKeyFrameMutex.unlock();

    // Smth unknown...
    if( SLAMEnabled )
    {
        // Lock the thread
        // Add the frame connected with Id to the list
        keyFrameGraph->idToKeyFrameMutex.lock();

        keyFrameGraph->idToKeyFrame.insert( std::make_pair( currentKeyFrame->id(),
                                                            currentKeyFrame         ) );
        // Unlock the thread
        keyFrameGraph->idToKeyFrameMutex.unlock();
    }

    // Visualization of the Key Frame (Vizualization 3D)
    if( continuousPCOutput && outputWrapper != 0 )
        outputWrapper->publishKeyframe( currentKeyFrame.get() );

    if (displayDepthMap || depthMapScreenshotFlag)
        debugDisplayDepthMap();

    // For debuging without mapping thread loop
    // doMappingIteration();

    // Ouput of the debugging information
    printf("Done Random initialization!\n");
}

// Processing of all "not first" frames
void SlamSystem::trackFrame(    uchar*          image               ,
                                unsigned int    frameID             ,
                                bool            blockUntilMapped    ,
                                double          timestamp               )
{
    // Create new frame (copying an input image)
    std::shared_ptr<Frame> trackingNewFrame(    new Frame(  frameID     ,
                                                            width       ,
                                                            height      ,
                                                            K           ,
                                                            timestamp   ,
                                                            image           )   );

    // If tracking isn't OK ???
    // "When we have the failure of synchronisation"
    if( !trackingIsGood )
    {
        // Refresh the current frame
        relocalizer.updateCurrentFrame( trackingNewFrame );

        // Lock the thread (which???? )
        unmappedTrackedFramesMutex.lock();
        // Notify to waiting one (about what ??? ) mappingThreadLoop
        unmappedTrackedFramesSignal.notify_one();
        // Unlock the thread
        unmappedTrackedFramesMutex.unlock();

        // Stop perfomance
        return;
    }

    // Protect the section ( currentKeyFrame )
    // Beginning of frame processing
    currentKeyFrameMutex.lock();

    // The condition is saved before work is started
    // createNewKeyFrame is changed in this function
    bool my_createNewKeyframe = createNewKeyFrame;

    // Find out what we check here !!!
    // ( The condition of trackingReference depends on other thread)
    if (    trackingReference->keyframe != currentKeyFrame.get()    ||
            currentKeyFrame->depthHasBeenUpdatedFlag    )
    {
        // Make appropriate changes (update data in trackingReference)
        // If tracking reference doesn't match to the current frame or depth map is updated
        trackingReference->importFrame( currentKeyFrame.get() );
        // Clear flag
        currentKeyFrame->depthHasBeenUpdatedFlag = false;
        // Update pointer
        trackingReferenceFrameSharedPT = currentKeyFrame;
    }

    // Make a reference to ... (check the usage)
    FramePoseStruct* trackingReferencePose = trackingReference->keyframe->pose;

    // Unlock
    currentKeyFrameMutex.unlock();

    // DO TRACKING & Show tracking result.
    if( enablePrintDebugInfo && printThreadingInfo )
        // Show the information
        printf( "TRACKING %d on %d\n", trackingNewFrame->id(), trackingReferencePose->frameID );

    // Lock an access to the data
    poseConsistencyMutex.lock_shared();

    // Find out what is it ???
    SE3 frameToReference_initialEstimate = se3FromSim3(
            trackingReferencePose->getCamToWorld().inverse() * keyFrameGraph->allFramePoses.back()->getCamToWorld());

    // Unlock the access
    poseConsistencyMutex.unlock_shared();

    // Probably for counting time for calculation
    struct timeval tv_start;
    struct timeval tv_end;

    // Store the time of the beginning
    gettimeofday( &tv_start, NULL );

    SE3 newRefToFrame_poseUpdate = tracker->trackFrame( trackingReference       ,
                                                        trackingNewFrame.get()  ,
                                                        frameToReference_initialEstimate    );

    // Store the time of finishing the calculation
    gettimeofday( &tv_end, NULL );

    // Count the time for function's execution
    msTrackFrame = 0.9 * msTrackFrame + 0.1 * ( (tv_end.tv_sec - tv_start.tv_sec) * 1000.0f + ( tv_end.tv_usec - tv_start.tv_usec )/ 1000.0f );
    // Frame counter incriment
    nTrackFrame++;

    // Statistics data
    tracking_lastResidual       = tracker->lastResidual;
    tracking_lastUsage          = tracker->pointUsage;
    tracking_lastGoodPerBad     = tracker->lastGoodCount / ( tracker->lastGoodCount + tracker->lastBadCount );
    tracking_lastGoodPerTotal   = tracker->lastGoodCount / ( trackingNewFrame->width(SE3TRACKING_MIN_LEVEL)*trackingNewFrame->height(SE3TRACKING_MIN_LEVEL));

    if( manualTrackingLossIndicated     ||          // Tracking lost state made manualy
        tracker->diverged               ||
        ( keyFrameGraph->keyframesAll.size() > INITIALIZATION_PHASE_COUNT && !tracker->trackingWasGood    )  )
    {
        printf( "TRACKING LOST for frame %d (%1.2f%% good Points, which is %1.2f%% of available points, %s)!\n",
                trackingNewFrame->id(),
                100*tracking_lastGoodPerTotal,
                100*tracking_lastGoodPerBad,
                tracker->diverged ? "DIVERGED" : "NOT DIVERGED");

        trackingReference->invalidate();

        trackingIsGood  = false;
        nextRelocIdx    = -1;

        unmappedTrackedFramesMutex.lock();
        unmappedTrackedFramesSignal.notify_one();
        unmappedTrackedFramesMutex.unlock();

        // Flag of manual reset of failure
        manualTrackingLossIndicated = false;
        return;
    }

    // If drawing of tracking is ON
    if(plotTracking)
    {
        Eigen::Matrix<float, 20, 1> data;
        data.setZero();
        data[0] = tracker->lastResidual;

        data[3] = tracker->lastGoodCount / (tracker->lastGoodCount + tracker->lastBadCount);
        data[4] = 4*tracker->lastGoodCount / (width*height);
        data[5] = tracker->pointUsage;

        data[6] = tracker->affineEstimation_a;
        data[7] = tracker->affineEstimation_b;
        // Publish data
        outputWrapper->publishDebugInfo(data);
    }

    // Append KF to KF graph
    keyFrameGraph->addFrame(trackingNewFrame.get());

    // Sim3 lastTrackedCamToWorld = mostCurrentTrackedFrame->getScaledCamToWorld();
    //  mostCurrentTrackedFrame->TrackingParent->getScaledCamToWorld() * sim3FromSE3(mostCurrentTrackedFrame->thisToParent_SE3TrackingResult, 1.0);
    if (outputWrapper != 0)
    {
        outputWrapper->publishTrackedFrame(trackingNewFrame.get());
    }

    // Keyframe selection
    latestTrackedFrame = trackingNewFrame;

    if (    !my_createNewKeyframe   &&
            currentKeyFrame->numMappedOnThisTotal > MIN_NUM_MAPPED  )
    {
        Sophus::Vector3d    dist    = newRefToFrame_poseUpdate.translation() * currentKeyFrame->meanIdepth;
        float               minVal  = fmin(0.2f + keyFrameGraph->keyframesAll.size() * 0.8f / INITIALIZATION_PHASE_COUNT,1.0f);

        if(keyFrameGraph->keyframesAll.size() < INITIALIZATION_PHASE_COUNT)
            minVal *= 0.7;

        lastTrackingClosenessScore = trackableKeyFrameSearch->getRefFrameScore(dist.dot(dist), tracker->pointUsage);

        if (lastTrackingClosenessScore > minVal)
        {
            createNewKeyFrame = true;

            if(enablePrintDebugInfo && printKeyframeSelectionInfo)
                printf(     "SELECT %d on %d! dist %.3f + usage %.3f = %.3f > 1\n",
                            trackingNewFrame->id(),
                            trackingNewFrame->getTrackingParent()->id(),
                            dist.dot(dist),
                            tracker->pointUsage,
                            trackableKeyFrameSearch->getRefFrameScore(  dist.dot(dist),
                                                                        tracker->pointUsage)    );
        }
        else
        {
            if( enablePrintDebugInfo && printKeyframeSelectionInfo )
                printf(     "SKIPPD %d on %d! dist %.3f + usage %.3f = %.3f > 1\n",
                            trackingNewFrame->id(),
                            trackingNewFrame->getTrackingParent()->id(),
                            dist.dot(dist),
                            tracker->pointUsage,
                            trackableKeyFrameSearch->getRefFrameScore(  dist.dot(dist),
                                                                        tracker->pointUsage )    );

        }
    }

    unmappedTrackedFramesMutex.lock();

    // Count how many frames are added to the current Key Frame (numMappedOnThisTotal)
    // Conditions for adding of the current frame
    if(         unmappedTrackedFrames.size() < 50   ||
            (   unmappedTrackedFrames.size() < 100  &&
                trackingNewFrame->getTrackingParent()->numMappedOnThisTotal < 10    )   )
        // Add new frame to deque.
        // TODO: else - skip frame ????
        unmappedTrackedFrames.push_back(trackingNewFrame);


    unmappedTrackedFramesSignal.notify_one();
    unmappedTrackedFramesMutex.unlock();

    // For debugging without mapping thread
    doMappingIteration();

    // Implement blocking
    if( blockUntilMapped && trackingIsGood )
    {
        boost::unique_lock<boost::mutex> lock( newFrameMappedMutex );

        // While buffer is not empty
        while( unmappedTrackedFrames.size() > 0 )
        {
            //printf("TRACKING IS BLOCKING, waiting for %d frames to finish mapping.\n", (int)unmappedTrackedFrames.size());
            newFrameMappedSignal.wait(lock);
        }

        lock.unlock();
    }

}

// Проверил - можно просто расскоментировать
//float SlamSystem::tryTrackSim3(
//		TrackingReference* A, TrackingReference* B,
//		int lvlStart, int lvlEnd,
//		bool useSSE,
//		Sim3 &AtoB, Sim3 &BtoA,
//		KFConstraintStruct* e1, KFConstraintStruct* e2 )
//{
//	BtoA = constraintTracker->trackFrameSim3(
//			A,
//			B->keyframe,
//			BtoA,
//			lvlStart,lvlEnd);
//	Matrix7x7 BtoAInfo = constraintTracker->lastSim3Hessian;
//	float BtoA_meanResidual = constraintTracker->lastResidual;
//	float BtoA_meanDResidual = constraintTracker->lastDepthResidual;
//	float BtoA_meanPResidual = constraintTracker->lastPhotometricResidual;
//	float BtoA_usage = constraintTracker->pointUsage;


//	if (constraintTracker->diverged ||
//		BtoA.scale() > 1 / Sophus::SophusConstants<sophusType>::epsilon() ||
//		BtoA.scale() < Sophus::SophusConstants<sophusType>::epsilon() ||
//		BtoAInfo(0,0) == 0 ||
//		BtoAInfo(6,6) == 0)
//	{
//		return 1e20;
//	}


//	AtoB = constraintTracker->trackFrameSim3(
//			B,
//			A->keyframe,
//			AtoB,
//			lvlStart,lvlEnd);
//	Matrix7x7 AtoBInfo = constraintTracker->lastSim3Hessian;
//	float AtoB_meanResidual = constraintTracker->lastResidual;
//	float AtoB_meanDResidual = constraintTracker->lastDepthResidual;
//	float AtoB_meanPResidual = constraintTracker->lastPhotometricResidual;
//	float AtoB_usage = constraintTracker->pointUsage;


//	if (constraintTracker->diverged ||
//		AtoB.scale() > 1 / Sophus::SophusConstants<sophusType>::epsilon() ||
//		AtoB.scale() < Sophus::SophusConstants<sophusType>::epsilon() ||
//		AtoBInfo(0,0) == 0 ||
//		AtoBInfo(6,6) == 0)
//	{
//		return 1e20;
//	}

//	// Propagate uncertainty (with d(a * b) / d(b) = Adj_a) and calculate Mahalanobis norm
//	Matrix7x7 datimesb_db = AtoB.cast<float>().Adj();
//	Matrix7x7 diffHesse = (AtoBInfo.inverse() + datimesb_db * BtoAInfo.inverse() * datimesb_db.transpose()).inverse();
//	Vector7 diff = (AtoB * BtoA).log().cast<float>();


//	float reciprocalConsistency = (diffHesse * diff).dot(diff);


//	if(e1 != 0 && e2 != 0)
//	{
//		e1->firstFrame = A->keyframe;
//		e1->secondFrame = B->keyframe;
//		e1->secondToFirst = BtoA;
//		e1->information = BtoAInfo.cast<double>();
//		e1->meanResidual = BtoA_meanResidual;
//		e1->meanResidualD = BtoA_meanDResidual;
//		e1->meanResidualP = BtoA_meanPResidual;
//		e1->usage = BtoA_usage;

//		e2->firstFrame = B->keyframe;
//		e2->secondFrame = A->keyframe;
//		e2->secondToFirst = AtoB;
//		e2->information = AtoBInfo.cast<double>();
//		e2->meanResidual = AtoB_meanResidual;
//		e2->meanResidualD = AtoB_meanDResidual;
//		e2->meanResidualP = AtoB_meanPResidual;
//		e2->usage = AtoB_usage;

//		e1->reciprocalConsistency = e2->reciprocalConsistency = reciprocalConsistency;
//	}

//	return reciprocalConsistency;
//}


// Проверил - можно просто расскоментировать
//void SlamSystem::testConstraint (
//		Frame* candidate,
//		KFConstraintStruct* &e1_out, KFConstraintStruct* &e2_out,
//		Sim3 candidateToFrame_initialEstimate,
//		float strictness)
//{
//	candidateTrackingReference->importFrame(candidate);

//	Sim3 FtoC = candidateToFrame_initialEstimate.inverse(), CtoF = candidateToFrame_initialEstimate;
//	Matrix7x7 FtoCInfo, CtoFInfo;

//	float err_level3 = tryTrackSim3(
//			newKFTrackingReference, candidateTrackingReference,	// A = frame; b = candidate
//			SIM3TRACKING_MAX_LEVEL-1, 3,
//			USESSE,
//			FtoC, CtoF);

//	if(err_level3 > 3000*strictness)
//	{
//		if(enablePrintDebugInfo && printConstraintSearchInfo)
//			printf("FAILE %d -> %d (lvl %d): errs (%.1f / - / -).",
//				newKFTrackingReference->frameID, candidateTrackingReference->frameID,
//				3,
//				sqrtf(err_level3));

//		e1_out = e2_out = 0;

//		newKFTrackingReference->keyframe->trackingFailed.insert(std::pair<Frame*,Sim3>(candidate, candidateToFrame_initialEstimate));
//		return;
//	}

//	float err_level2 = tryTrackSim3(
//			newKFTrackingReference, candidateTrackingReference,	// A = frame; b = candidate
//			2, 2,
//			USESSE,
//			FtoC, CtoF);

//	if(err_level2 > 4000*strictness)
//	{
//		if(enablePrintDebugInfo && printConstraintSearchInfo)
//			printf("FAILE %d -> %d (lvl %d): errs (%.1f / %.1f / -).",
//				newKFTrackingReference->frameID, candidateTrackingReference->frameID,
//				2,
//				sqrtf(err_level3), sqrtf(err_level2));

//		e1_out = e2_out = 0;
//		newKFTrackingReference->keyframe->trackingFailed.insert(std::pair<Frame*,Sim3>(candidate, candidateToFrame_initialEstimate));
//		return;
//	}

//	e1_out = new KFConstraintStruct();
//	e2_out = new KFConstraintStruct();


//	float err_level1 = tryTrackSim3(
//			newKFTrackingReference, candidateTrackingReference,	// A = frame; b = candidate
//			1, 1,
//			USESSE,
//			FtoC, CtoF, e1_out, e2_out);

//	if(err_level1 > 6000*strictness)
//	{
//		if(enablePrintDebugInfo && printConstraintSearchInfo)
//			printf("FAILE %d -> %d (lvl %d): errs (%.1f / %.1f / %.1f).",
//					newKFTrackingReference->frameID, candidateTrackingReference->frameID,
//					1,
//					sqrtf(err_level3), sqrtf(err_level2), sqrtf(err_level1));

//		delete e1_out;
//		delete e2_out;
//		e1_out = e2_out = 0;
//		newKFTrackingReference->keyframe->trackingFailed.insert(std::pair<Frame*,Sim3>(candidate, candidateToFrame_initialEstimate));
//		return;
//	}


//	if(enablePrintDebugInfo && printConstraintSearchInfo)
//		printf("ADDED %d -> %d: errs (%.1f / %.1f / %.1f).",
//			newKFTrackingReference->frameID, candidateTrackingReference->frameID,
//			sqrtf(err_level3), sqrtf(err_level2), sqrtf(err_level1));


//	const float kernelDelta = 5 * sqrt(6000*loopclosureStrictness);
//	e1_out->robustKernel = new g2o::RobustKernelHuber();
//	e1_out->robustKernel->setDelta(kernelDelta);
//	e2_out->robustKernel = new g2o::RobustKernelHuber();
//	e2_out->robustKernel->setDelta(kernelDelta);
//}

// Проверил - можно просто расскоментировать
//int SlamSystem::findConstraintsForNewKeyFrames(Frame* newKeyFrame, bool forceParent, bool useFABMAP, float closeCandidatesTH)
//{
//    if(!newKeyFrame->hasTrackingParent())
//    {
//        newConstraintMutex.lock();
//        keyFrameGraph->addKeyFrame(newKeyFrame);
//        newConstraintAdded = true;
//        newConstraintCreatedSignal.notify_all();
//        newConstraintMutex.unlock();
//        return 0;
//    }

//	if(!forceParent && (newKeyFrame->lastConstraintTrackedCamToWorld * newKeyFrame->getScaledCamToWorld().inverse()).log().norm() < 0.01)
//		return 0;


//	newKeyFrame->lastConstraintTrackedCamToWorld = newKeyFrame->getScaledCamToWorld();

//	// =============== Get all potential candidates and their initial relative pose. =================
//    std::vector<KFConstraintStruct*, Eigen::aligned_allocator<KFConstraintStruct*> > constraints;
//    Frame* fabMapResult = 0;
//    std::unordered_set<Frame*, std::hash<Frame*>, std::equal_to<Frame*>,
//        Eigen::aligned_allocator< Frame* > > candidates = trackableKeyFrameSearch->findCandidates(newKeyFrame, fabMapResult, useFABMAP, closeCandidatesTH);
//    std::map< Frame*, Sim3, std::less<Frame*>, Eigen::aligned_allocator<std::pair<Frame*, Sim3> > > candidateToFrame_initialEstimateMap;


//	// erase the ones that are already neighbours.
//    for(std::unordered_set<Frame*>::iterator c = candidates.begin(); c != candidates.end();)
//    {
//        if(newKeyFrame->neighbors.find(*c) != newKeyFrame->neighbors.end())
//        {
//            if(enablePrintDebugInfo && printConstraintSearchInfo)
//                printf("SKIPPING %d on %d cause it already exists as constraint.\n", (*c)->id(), newKeyFrame->id());
//            c = candidates.erase(c);
//        }
//        else
//            ++c;
//    }

//    poseConsistencyMutex.lock_shared();
//    for (Frame* candidate : candidates)
//    {
//        Sim3 candidateToFrame_initialEstimate = newKeyFrame->getScaledCamToWorld().inverse() * candidate->getScaledCamToWorld();
//        candidateToFrame_initialEstimateMap[candidate] = candidateToFrame_initialEstimate;
//    }

//    std::unordered_map<Frame*, int> distancesToNewKeyFrame;

//    if(newKeyFrame->hasTrackingParent())
//        keyFrameGraph->calculateGraphDistancesToFrame(newKeyFrame->getTrackingParent(), &distancesToNewKeyFrame);

//	poseConsistencyMutex.unlock_shared();

//	// =============== Distinguish between close and "far" candidates in Graph =================
//	// Do a first check on trackability of close candidates.
//	std::unordered_set<Frame*, std::hash<Frame*>, std::equal_to<Frame*>,
//		Eigen::aligned_allocator< Frame* > > closeCandidates;
//	std::vector<Frame*, Eigen::aligned_allocator<Frame*> > farCandidates;

//    Frame* parent = newKeyFrame->hasTrackingParent() ? newKeyFrame->getTrackingParent() : 0;

//	int closeFailed = 0;
//	int closeInconsistent = 0;

//	SO3 disturbance = SO3::exp(Sophus::Vector3d(0.05,0,0));

//    for (Frame* candidate : candidates)
//    {
//        if (candidate->id() == newKeyFrame->id())
//            continue;
//        if(!candidate->pose->isInGraph)
//            continue;
//        if(newKeyFrame->hasTrackingParent() && candidate == newKeyFrame->getTrackingParent())
//            continue;
//        if(candidate->idxInKeyframes < INITIALIZATION_PHASE_COUNT)
//            continue;

//        SE3 c2f_init = se3FromSim3(candidateToFrame_initialEstimateMap[candidate].inverse()).inverse();

//        c2f_init.so3() = c2f_init.so3() * disturbance;
//        SE3 c2f = constraintSE3Tracker->trackFrameOnPermaref(candidate, newKeyFrame, c2f_init);
//        if(!constraintSE3Tracker->trackingWasGood) {closeFailed++; continue;}

//        SE3 f2c_init = se3FromSim3(candidateToFrame_initialEstimateMap[candidate]).inverse();
//        f2c_init.so3() = disturbance * f2c_init.so3();
//        SE3 f2c = constraintSE3Tracker->trackFrameOnPermaref(newKeyFrame, candidate, f2c_init);
//        if(!constraintSE3Tracker->trackingWasGood) {closeFailed++; continue;}

//        if((f2c.so3() * c2f.so3()).log().norm() >= 0.09) {closeInconsistent++; continue;}

//        closeCandidates.insert(candidate);
//    }

//    int farFailed = 0;
//    int farInconsistent = 0;
//    for (Frame* candidate : candidates)
//    {
//        if (candidate->id() == newKeyFrame->id())
//            continue;
//        if(!candidate->pose->isInGraph)
//            continue;
//        if(newKeyFrame->hasTrackingParent() && candidate == newKeyFrame->getTrackingParent())
//            continue;
//        if(candidate->idxInKeyframes < INITIALIZATION_PHASE_COUNT)
//            continue;

//        if(candidate == fabMapResult)
//        {
//            farCandidates.push_back(candidate);
//            continue;
//        }

//        if(distancesToNewKeyFrame.at(candidate) < 4)
//            continue;

//        farCandidates.push_back(candidate);
//    }

//    int closeAll    = closeCandidates.size();
//    int farAll      = farCandidates.size();

//	// Erase the ones that we tried already before (close)
//    for(std::unordered_set<Frame*>::iterator c = closeCandidates.begin(); c != closeCandidates.end();)
//    {
//        if(newKeyFrame->trackingFailed.find(*c) == newKeyFrame->trackingFailed.end())
//        {
//            ++c;
//            continue;
//        }
//        auto range = newKeyFrame->trackingFailed.equal_range(*c);


//        bool skip = false;
//        Sim3 f2c = candidateToFrame_initialEstimateMap[*c].inverse();
//        for (auto it = range.first; it != range.second; ++it)
//        {
//            if((f2c * it->second).log().norm() < 0.1)
//            {
//                skip=true;
//                break;
//            }
//        }

//        if(skip)
//        {
//            if(enablePrintDebugInfo && printConstraintSearchInfo)
//                printf("SKIPPING %d on %d (NEAR), cause we already have tried it.\n", (*c)->id(), newKeyFrame->id());
//            c = closeCandidates.erase(c);
//        }
//        else
//            ++c;
//    }

//    // erase the ones that are already neighbours (far)
//    for(unsigned int i=0;i<farCandidates.size();i++)
//    {
//        if(newKeyFrame->trackingFailed.find(farCandidates[i]) == newKeyFrame->trackingFailed.end())
//            continue;

//        auto range = newKeyFrame->trackingFailed.equal_range(farCandidates[i]);

//        bool skip = false;
//        for (auto it = range.first; it != range.second; ++it)
//        {
//            if((it->second).log().norm() < 0.2)
//            {
//                skip=true;
//                break;
//            }
//        }

//        if(skip)
//        {
//            if(enablePrintDebugInfo && printConstraintSearchInfo)
//                printf("SKIPPING %d on %d (FAR), cause we already have tried it.\n", farCandidates[i]->id(), newKeyFrame->id());
//            farCandidates[i] = farCandidates.back();
//            farCandidates.pop_back();
//            i--;
//        }
//    }


//    if (enablePrintDebugInfo && printConstraintSearchInfo)
//        printf("Final Loop-Closure Candidates: %d / %d close (%d failed, %d inconsistent) + %d / %d far (%d failed, %d inconsistent) = %d\n",
//                (int)closeCandidates.size(),closeAll, closeFailed, closeInconsistent,
//                (int)farCandidates.size(), farAll, farFailed, farInconsistent,
//                (int)closeCandidates.size() + (int)farCandidates.size());



//     //=============== Limit number of close candidates ===============
//     // while too many, remove the one with the highest connectivity.
//    while((int)closeCandidates.size() > maxLoopClosureCandidates)
//    {
//        Frame* worst = 0;
//        int worstNeighbours = 0;
//        for(Frame* f : closeCandidates)
//        {
//            int neightboursInCandidates = 0;
//            for(Frame* n : f->neighbors)
//                if(closeCandidates.find(n) != closeCandidates.end())
//                    neightboursInCandidates++;

//            if(neightboursInCandidates > worstNeighbours || worst == 0)
//            {
//                worst = f;
//                worstNeighbours = neightboursInCandidates;
//            }
//        }

//        closeCandidates.erase(worst);
//    }

//     //=============== Limit number of far candidates ===============
//     // delete randomly
//    int maxNumFarCandidates = (maxLoopClosureCandidates +1) / 2;
//    if(maxNumFarCandidates < 5) maxNumFarCandidates = 5;
//    while((int)farCandidates.size() > maxNumFarCandidates)
//    {
//        int toDelete = rand() % farCandidates.size();

//        if(farCandidates[toDelete] != fabMapResult)
//        {
//            farCandidates[toDelete] = farCandidates.back();
//            farCandidates.pop_back();
//        }
//    }

//   //=============== TRACK! ===============

//    // Make tracking reference for newKeyFrame.
//    newKFTrackingReference->importFrame(newKeyFrame);

//    for (Frame* candidate : closeCandidates)
//    {
//        KFConstraintStruct* e1=0;
//        KFConstraintStruct* e2=0;

//        testConstraint(
//                candidate, e1, e2,
//                candidateToFrame_initialEstimateMap[candidate],
//                loopclosureStrictness);

//        if(enablePrintDebugInfo && printConstraintSearchInfo)
//            printf(" CLOSE (%d)\n", distancesToNewKeyFrame.at(candidate));

//        if(e1 != 0)
//        {
//            constraints.push_back(e1);
//            constraints.push_back(e2);

//            // delete from far candidates if it's in there.
//            for(unsigned int k=0;k<farCandidates.size();k++)
//            {
//                if(farCandidates[k] == candidate)
//                {
//                    if(enablePrintDebugInfo && printConstraintSearchInfo)
//                        printf(" DELETED %d from far, as close was successful!\n", candidate->id());

//                    farCandidates[k] = farCandidates.back();
//                    farCandidates.pop_back();
//                }
//            }
//        }
//    }

//    for (Frame* candidate : farCandidates)
//    {
//        KFConstraintStruct* e1=0;
//        KFConstraintStruct* e2=0;

//        testConstraint(
//                candidate, e1, e2,
//                Sim3(),
//                loopclosureStrictness);

//        if(enablePrintDebugInfo && printConstraintSearchInfo)
//            printf(" FAR (%d)\n", distancesToNewKeyFrame.at(candidate));

//        if(e1 != 0)
//        {
//            constraints.push_back(e1);
//            constraints.push_back(e2);
//        }
//    }

//    if(parent != 0 && forceParent)
//    {
//        KFConstraintStruct* e1=0;
//        KFConstraintStruct* e2=0;
//        testConstraint(
//                parent, e1, e2,
//                candidateToFrame_initialEstimateMap[parent],
//                100);
//        if(enablePrintDebugInfo && printConstraintSearchInfo)
//            printf(" PARENT (0)\n");

//        if(e1 != 0)
//        {
//            constraints.push_back(e1);
//            constraints.push_back(e2);
//        }
//        else
//        {
//            float downweightFac = 5;
//            const float kernelDelta = 5 * sqrt(6000*loopclosureStrictness) / downweightFac;
//            printf("warning: reciprocal tracking on new frame failed badly, added odometry edge (Hacky).\n");

//            poseConsistencyMutex.lock_shared();
//            constraints.push_back(new KFConstraintStruct());
//            constraints.back()->firstFrame = newKeyFrame;
//            constraints.back()->secondFrame = newKeyFrame->getTrackingParent();
//            constraints.back()->secondToFirst = constraints.back()->firstFrame->getScaledCamToWorld().inverse() * constraints.back()->secondFrame->getScaledCamToWorld();
//            constraints.back()->information  <<
//                    0.8098,-0.1507,-0.0557, 0.1211, 0.7657, 0.0120, 0,
//                    -0.1507, 2.1724,-0.1103,-1.9279,-0.1182, 0.1943, 0,
//                    -0.0557,-0.1103, 0.2643,-0.0021,-0.0657,-0.0028, 0.0304,
//                     0.1211,-1.9279,-0.0021, 2.3110, 0.1039,-0.0934, 0.0005,
//                     0.7657,-0.1182,-0.0657, 0.1039, 1.0545, 0.0743,-0.0028,
//                     0.0120, 0.1943,-0.0028,-0.0934, 0.0743, 0.4511, 0,
//                    0,0, 0.0304, 0.0005,-0.0028, 0, 0.0228;
//            constraints.back()->information *= (1e9/(downweightFac*downweightFac));

//            constraints.back()->robustKernel = new g2o::RobustKernelHuber();
//            constraints.back()->robustKernel->setDelta(kernelDelta);

//            constraints.back()->meanResidual = 10;
//            constraints.back()->meanResidualD = 10;
//            constraints.back()->meanResidualP = 10;
//            constraints.back()->usage = 0;

//            poseConsistencyMutex.unlock_shared();
//        }
//    }

//    newConstraintMutex.lock();

//    keyFrameGraph->addKeyFrame(newKeyFrame);
//    for(unsigned int i=0;i<constraints.size();i++)
//        keyFrameGraph->insertConstraint(constraints[i]);

//    newConstraintAdded = true;
//    newConstraintCreatedSignal.notify_all();
//    newConstraintMutex.unlock();

//    newKFTrackingReference->invalidate();
//    candidateTrackingReference->invalidate();

//    return constraints.size();
//}

// Проверил - можно просто расскоментировать
//bool SlamSystem::optimizationIteration(int itsPerTry, float minChange)
//{
//	struct timeval tv_start, tv_end;

//	gettimeofday(&tv_start, NULL);

//	g2oGraphAccessMutex.lock();

//	// lock new elements buffer & take them over.
//	newConstraintMutex.lock();
//	keyFrameGraph->addElementsFromBuffer();
//	newConstraintMutex.unlock();

//	// Do the optimization. This can take quite some time!
//	int its = keyFrameGraph->optimize(itsPerTry);

//	// save the optimization result.
//	poseConsistencyMutex.lock_shared();

//	keyFrameGraph->keyframesAllMutex.lock_shared();
//	float maxChange = 0;
//	float sumChange = 0;
//	float sum = 0;

//	for(size_t i=0;i<keyFrameGraph->keyframesAll.size(); i++)
//	{
//		// set edge error sum to zero
//		keyFrameGraph->keyframesAll[i]->edgeErrorSum = 0;
//		keyFrameGraph->keyframesAll[i]->edgesNum = 0;

//        if(!keyFrameGraph->keyframesAll[i]->pose->isInGraph)
//            continue;

//		// get change from last optimization
//		Sim3 a = keyFrameGraph->keyframesAll[i]->pose->graphVertex->estimate();
//		Sim3 b = keyFrameGraph->keyframesAll[i]->getScaledCamToWorld();
//		Sophus::Vector7f diff = (a*b.inverse()).log().cast<float>();

//		for(int j=0;j<7;j++)
//		{
//			float d = fabsf((float)(diff[j]));
//			if(d > maxChange) maxChange = d;
//			sumChange += d;
//		}
//		sum +=7;

//		// set change
//		keyFrameGraph->keyframesAll[i]->pose->setPoseGraphOptResult(
//				keyFrameGraph->keyframesAll[i]->pose->graphVertex->estimate());

//		// add error
//		for(auto edge : keyFrameGraph->keyframesAll[i]->pose->graphVertex->edges())
//		{
//			keyFrameGraph->keyframesAll[i]->edgeErrorSum += ((EdgeSim3*)(edge))->chi2();
//			keyFrameGraph->keyframesAll[i]->edgesNum++;
//		}
//	}

//	haveUnmergedOptimizationOffset = true;

//	keyFrameGraph->keyframesAllMutex.unlock_shared();

//	poseConsistencyMutex.unlock_shared();

//	g2oGraphAccessMutex.unlock();

//	if(enablePrintDebugInfo && printOptimizationInfo)
//		printf("did %d optimization iterations. Max Pose Parameter Change: %f; avgChange: %f. %s\n", its, maxChange, sumChange / sum,
//				maxChange > minChange && its == itsPerTry ? "continue optimizing":"Waiting for addition to graph.");


//	gettimeofday(&tv_end, NULL);
//	msOptimizationIteration = 0.9*msOptimizationIteration + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
//	nOptimizationIteration++;

//    return maxChange > minChange && its == itsPerTry;
//}

// Проверил - можно просто расскоментировать
//void SlamSystem::optimizeGraph()
//{
//	boost::unique_lock<boost::mutex> g2oLock(g2oGraphAccessMutex);
//	keyFrameGraph->optimize(1000);
//	g2oLock.unlock();
//	mergeOptimizationOffset();
//}

// Проверил - можно просто расскоментировать
//SE3 SlamSystem::getCurrentPoseEstimate()
//{
//	SE3 camToWorld = SE3();

//	keyFrameGraph->allFramePosesMutex.lock_shared();

//	if(keyFrameGraph->allFramePoses.size() > 0)
//		camToWorld = se3FromSim3(keyFrameGraph->allFramePoses.back()->getCamToWorld());

//    keyFrameGraph->allFramePosesMutex.unlock_shared();

//	return camToWorld;
//}

// Проверил - можно просто расскоментировать
//std::vector<FramePoseStruct*, Eigen::aligned_allocator<FramePoseStruct*> > SlamSystem::getAllPoses()
//{
//    // Возвращает все положения
//	return keyFrameGraph->allFramePoses;
//}
