#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Atlas.h"
#include "Frame.h"
#include "FrameDrawer.h"
#include "ImuTypes.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Settings.h"
#include "System.h"
#include "Viewer.h"

#include "GeometricCamera.h"

#include <mutex>
#include <unordered_set>

namespace ORB_SLAM3 {

class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
class Settings;

class Tracking {

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Tracking(
        System *pSys,
        ORBVocabulary *pVoc,
        FrameDrawer *pFrameDrawer,
        MapDrawer *pMapDrawer,
        Atlas *pAtlas,
        KeyFrameDatabase *pKFDB,
        const string &strSettingPath,
        const int sensor,
        Settings *settings,
        const string &_nameSeq = std::string());

    ~Tracking();

    // Parse the config file
    bool ParseCamParamFile(cv::FileStorage &fSettings);
    bool ParseORBParamFile(cv::FileStorage &fSettings);
    bool ParseIMUParamFile(cv::FileStorage &fSettings);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    Sophus::SE3f GrabImageStereo(
        const cv::Mat &imRectLeft,
        const cv::Mat &imRectRight,
        const double &timestamp,
        string filename);
    Sophus::SE3f GrabImageRGBD(
        const cv::Mat &imRGB,
        const cv::Mat &imD,
        const double &timestamp,
        string filename);
    Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);
    Sophus::SE3f GrabImageMulti(
        const cv::Mat &imLeft,
        const cv::Mat &imRight,
        const cv::Mat &imSideLeft,
        const cv::Mat &imSideRight,
        const cv::Mat &depthUdLeft,
        const cv::Mat &depthUdRight,
        const cv::Mat &depthUdSideLeft,
        const cv::Mat &depthUdSideRight,
        const double &timestamp,
        string filename);

    void GrabImuData(const IMU::Point &imuMeasurement);

    void SetLocalMapper(LocalMapping *pLocalMapper);
    void SetLoopClosing(LoopClosing *pLoopClosing);
    void SetViewer(Viewer *pViewer);
    void SetStepByStep(bool bSet);
    bool GetStepByStep();

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the
    // camera.
    void InformOnlyTracking(const bool &flag);

    void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame *pCurrentKeyFrame);
    KeyFrame *GetLastKeyFrame() { return mpLastKeyFrame; }

    void CreateMapInAtlas();
    // std::mutex mMutexTracks;

    //--
    void NewDataset();
    int GetNumberDataset();
    int GetMatchesInliers();

    // DEBUG
    void
    SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder = "");
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map *pMap);

    float GetImageScale();

#ifdef REGISTER_LOOP
    void RequestStop();
    bool isStopped();
    void Release();
    bool stopRequested();
#endif

  public:
    // Tracking states
    enum eTrackingState {
        SYSTEM_NOT_READY = -1,
        NO_IMAGES_YET = 0,
        NOT_INITIALIZED = 1,
        OK = 2,
        RECENTLY_LOST = 3,
        LOST = 4,
        OK_KLT = 5
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    Frame mLastFrame;

    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<Sophus::SE3f> mlRelativeFramePoses;
    list<KeyFrame *> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // frames with estimated pose
    int mTrackedFr;
    bool mbStep;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset(bool bLocMap = false);
    void ResetActiveMap(bool bLocMap = false);

    bool mbleft{true}, mbright{true}, mbsideleft{true}, mbsideright{true};

    float mMeanTrack;
    bool mbInitWith3KFs;
    double t0;    // time-stamp of first read frame
    double t0vis; // time-stamp of first inserted keyframe
    double t0IMU; // time-stamp of IMU initialization
    bool mFastInit = false;

    vector<MapPoint *> GetLocalMapMPS();

    bool mbWriteStats;

#ifdef REGISTER_TIMES
    void LocalMapStats2File();
    void TrackStats2File();
    void PrintTimeStats();

    vector<double> vdRectStereo_ms;
    vector<double> vdResizeImage_ms;
    vector<double> vdORBExtract_ms;
    vector<double> vdStereoMatch_ms;
    vector<double> vdIMUInteg_ms;
    vector<double> vdPosePred_ms;
    vector<double> vdLMTrack_ms;
    vector<double> vdNewKF_ms;
    vector<double> vdTrackTotal_ms;
#endif

  protected:
    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for Multi fisheye
    void MultiInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    // void CreateNewMapPoints();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();
    bool PredictStateIMU();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // Perform preintegration from last frame
    void PreintegrateIMU();

    // Reset IMU biases and compute frame velocity
    void ResetFrameIMU();

    bool mbMapUpdated;

    // Imu preintegration from last frame
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;

    // Queue of IMU measurements between frames
    std::list<IMU::Point> mlQueueImuData;

    // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
    std::vector<IMU::Point> mvImuFromLastFrame;
    std::mutex mMutexImuQueue;

    // Imu calibration parameters
    IMU::Calib *mpImuCalib;

    // Last Bias Estimation (at keyframe creation)
    IMU::Bias mLastBias;

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal
    // points. In that case we are doing visual odometry. The system will try to do relocalization
    // to recover "zero-drift" localization to the map.
    bool mbVO;

    // Other Thread Pointers
    LocalMapping *mpLocalMapper;
    LoopClosing *mpLoopClosing;

    // ORB
    ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor *mpORBextractorSideLeft, *mpORBextractorSideRight;
    ORBextractor *mpIniORBextractor;

    // BoW
    ORBVocabulary *mpORBVocabulary;
    KeyFrameDatabase *mpKeyFrameDB;

    // Initalization (only for monocular)
    bool mbReadyToInitializate;
    bool mbSetInit;

    // Local Map
    KeyFrame *mpReferenceKF;
    std::vector<KeyFrame *> mvpLocalKeyFrames;
    std::vector<MapPoint *> mvpLocalMapPoints;

    // System
    System *mpSystem;

    // Drawers
    Viewer *mpViewer;
    FrameDrawer *mpFrameDrawer;
    MapDrawer *mpMapDrawer;
    bool bStepByStep;

    // Atlas
    Atlas *mpAtlas;

    // Calibration matrix
    cv::Mat mK;
    Eigen::Matrix3f mK_;
    cv::Mat mDistCoef;
    float mbf;
    float mImageScale;

    float mImuFreq;
    double mImuPer;
    bool mInsertKFsLost;

    // New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    int mnFirstImuFrameId;
    int mnFramesToResetIMU;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    // Current matches in frame
    int mnMatchesInliers;

    // Last Frame, KeyFrame and Relocalisation Info
    KeyFrame *mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;
    double mTimeStampLost;
    double time_recently_lost;

    unsigned int mnFirstFrameId;
    unsigned int mnInitialFrameId;
    unsigned int mnLastInitFrameId;

    bool mbCreatedMap;

    // Motion Model
    bool mbVelocity{false};
    Sophus::SE3f mVelocity;

    // Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint *> mlpTemporalPoints;

    // int nMapChangeIndex;

    int mnNumDataset;

    ofstream f_track_stats;

    ofstream f_track_times;
    double mTime_PreIntIMU;
    double mTime_PosePred;
    double mTime_LocalMapTrack;
    double mTime_NewKF_Dec;

    GeometricCamera *mpCamera, *mpCamera2;
    GeometricCamera *mpCamera3, *mpCamera4;

    int initID, lastID;

    Sophus::SE3f mTlr;
    Sophus::SE3f mTlsl;
    Sophus::SE3f mTlsr;

    void newParameterLoader(Settings *settings);

#ifdef REGISTER_LOOP
    bool Stop();

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;
#endif

  public:
    cv::Mat mImLeft, mImRight, mImSideLeft, mImSideRight;
};

} // namespace ORB_SLAM3

#endif // TRACKING_H
