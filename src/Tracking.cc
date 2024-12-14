#include "Tracking.h"

#include "Converter.h"
#include "FrameDrawer.h"
#include "G2oTypes.h"
#include "GeometricTools.h"
#include "KannalaBrandt8.h"
#include "MLPnPsolver.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Pinhole.h"

#include <iostream>

#include <mutex>

using namespace std;

namespace ORB_SLAM3 {

Tracking::Tracking(
    System *pSys,
    ORBVocabulary *pVoc,
    FrameDrawer *pFrameDrawer,
    MapDrawer *pMapDrawer,
    Atlas *pAtlas,
    KeyFrameDatabase *pKFDB,
    const string &strSettingPath,
    const int sensor,
    Settings *settings,
    const string &_nameSeq)
    : mState(NO_IMAGES_YET),
      mSensor(sensor),
      mTrackedFr(0),
      mbStep(false),
      mbOnlyTracking(false),
      mbMapUpdated(false),
      mbVO(false),
      mpORBVocabulary(pVoc),
      mpKeyFrameDB(pKFDB),
      mbReadyToInitializate(false),
      mpSystem(pSys),
      mpViewer(NULL),
      bStepByStep(false),
      mpFrameDrawer(pFrameDrawer),
      mpMapDrawer(pMapDrawer),
      mpAtlas(pAtlas),
      mnLastRelocFrameId(0),
      time_recently_lost(5.0),
      mnInitialFrameId(0),
      mbCreatedMap(false),
      mnFirstFrameId(0),
      mpCamera2(nullptr),
      mpLastKeyFrame(static_cast<KeyFrame *>(NULL)) {

    newParameterLoader(settings);

    initID = 0;
    lastID = 0;
    mbInitWith3KFs = false;
    mnNumDataset = 0;

    vector<GeometricCamera *> vpCams = mpAtlas->GetAllCameras();
    std::cout << "There are " << vpCams.size() << " cameras in the atlas" << std::endl;
    for (GeometricCamera *pCam : vpCams) {
        std::cout << "Camera " << pCam->GetId();
        if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
            std::cout << " is pinhole" << std::endl;
        } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
            std::cout << " is fisheye" << std::endl;
        } else {
            std::cout << " is unknown" << std::endl;
        }
    }
}

Tracking::~Tracking() {
    // f_track_stats.close();
}

void Tracking::newParameterLoader(Settings *settings) {
    mpCamera = settings->camera1();
    mpCamera = mpAtlas->AddCamera(mpCamera);

    if (settings->needToUndistort()) {
        mDistCoef = settings->camera1DistortionCoef();
    } else {
        mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
    }

    // TODO: missing image scaling and rectification
    mImageScale = 1.0f;

    mK = cv::Mat::eye(3, 3, CV_32F);
    mK.at<float>(0, 0) = mpCamera->getParameter(0);
    mK.at<float>(1, 1) = mpCamera->getParameter(1);
    mK.at<float>(0, 2) = mpCamera->getParameter(2);
    mK.at<float>(1, 2) = mpCamera->getParameter(3);

    mK_.setIdentity();
    mK_(0, 0) = mpCamera->getParameter(0);
    mK_(1, 1) = mpCamera->getParameter(1);
    mK_(0, 2) = mpCamera->getParameter(2);
    mK_(1, 2) = mpCamera->getParameter(3);

    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO ||
        mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI) {
        mpCamera2 = settings->camera2();
        mpCamera2 = mpAtlas->AddCamera(mpCamera2);

        mTlr = settings->Tlr();

        mpFrameDrawer->both = true;
    }

    if (mSensor == System::STEREO || mSensor == System::RGBD || mSensor == System::IMU_STEREO ||
        mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI) {
        mbf = settings->bf();
        mThDepth = settings->b() * settings->thDepth();
    }

    if (mSensor == System::RGBD || mSensor == System::IMU_RGBD) {
        mDepthMapFactor = settings->depthMapFactor();
        if (fabs(mDepthMapFactor) < 1e-5)
            mDepthMapFactor = 1;
        else
            mDepthMapFactor = 1.0f / mDepthMapFactor;
    }

    if (mSensor == System::IMU_MULTI) {
        mpCamera3 = settings->camera3();
        mpCamera3 = mpAtlas->AddCamera(mpCamera3);
        mpCamera4 = settings->camera4();
        mpCamera4 = mpAtlas->AddCamera(mpCamera4);
        mpCamera5 = settings->camera5();
        mpCamera5 = mpAtlas->AddCamera(mpCamera5);
        mpCamera6 = settings->camera6();
        mpCamera6 = mpAtlas->AddCamera(mpCamera6);
        mpCamera7 = settings->camera7();
        mpCamera7 = mpAtlas->AddCamera(mpCamera7);
        mpCamera8 = settings->camera8();
        mpCamera8 = mpAtlas->AddCamera(mpCamera8);
        mTlsl = settings->Tlsl();
        mTlsr = settings->Tlsr();
    }

    mMinFrames = 0;
    mMaxFrames = settings->fps();
    mbRGB = settings->rgb();

    // ORB parameters
    int nFeatures = settings->nFeatures();
    int nLevels = settings->nLevels();
    int fIniThFAST = settings->initThFAST();
    int fMinThFAST = settings->minThFAST();
    float fScaleFactor = settings->scaleFactor();

    mpORBextractorLeft =
        new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    mpORBextractorLeft2 =
        new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO ||
        mSensor == System::IMU_MULTI) {
        mpORBextractorRight =
            new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        mpORBextractorRight2 =
            new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    }

    if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
        mpIniORBextractor =
            new ORBextractor(5 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if (mSensor == System::IMU_MULTI) {
        mpORBextractorSideLeft =
            new ORBextractor(nFeatures * 2, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        mpORBextractorSideRight =
            new ORBextractor(nFeatures * 2, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        mpORBextractorSideLeft2 =
            new ORBextractor(nFeatures * 2, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        mpORBextractorSideRight2 =
            new ORBextractor(nFeatures * 2, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    }

    // IMU parameters
    Sophus::SE3f Tbc = settings->Tbc();
    mInsertKFsLost = settings->insertKFsWhenLost();
    mImuFreq = settings->imuFrequency();
    mImuPer = 0.001; // 1.0 / (double) mImuFreq;     //TODO: ESTO ESTA BIEN?
    float Ng = settings->noiseGyro();
    float Na = settings->noiseAcc();
    float Ngw = settings->gyroWalk();
    float Naw = settings->accWalk();

    const float sf = sqrt(mImuFreq);
    mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) { mpLocalMapper = pLocalMapper; }

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing) { mpLoopClosing = pLoopClosing; }

void Tracking::SetViewer(Viewer *pViewer) { mpViewer = pViewer; }

void Tracking::SetStepByStep(bool bSet) { bStepByStep = bSet; }

bool Tracking::GetStepByStep() { return bStepByStep; }

Sophus::SE3f Tracking::GrabImageStereo(
    const cv::Mat &imRectLeft,
    const cv::Mat &imRectRight,
    const double &timestamp,
    string filename) {
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;
    mImLeft = imRectLeft;
    mImRight = imRectRight;

    if (mImGray.channels() == 3) {
        // cout << "Image with 3 channels" << endl;
        if (mbRGB) {
            cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
        } else {
            cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
        }
    } else if (mImGray.channels() == 4) {
        // cout << "Image with 4 channels" << endl;
        if (mbRGB) {
            cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
        } else {
            cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
        }
    }

    // cout << "Incoming frame creation" << endl;

    if (mSensor == System::STEREO && !mpCamera2)
        mCurrentFrame = Frame(
            mImGray,
            imGrayRight,
            timestamp,
            mpORBextractorLeft,
            mpORBextractorRight,
            mpORBVocabulary,
            mK,
            mDistCoef,
            mbf,
            mThDepth,
            mpCamera);
    else if (mSensor == System::STEREO && mpCamera2)
        mCurrentFrame = Frame(
            mImGray,
            imGrayRight,
            timestamp,
            mpORBextractorLeft,
            mpORBextractorRight,
            mpORBVocabulary,
            mK,
            mDistCoef,
            mbf,
            mThDepth,
            mpCamera,
            mpCamera2,
            mTlr);
    else if (mSensor == System::IMU_STEREO && !mpCamera2)
        mCurrentFrame = Frame(
            mImGray,
            imGrayRight,
            timestamp,
            mpORBextractorLeft,
            mpORBextractorRight,
            mpORBVocabulary,
            mK,
            mDistCoef,
            mbf,
            mThDepth,
            mpCamera,
            &mLastFrame,
            *mpImuCalib);
    else if (mSensor == System::IMU_STEREO && mpCamera2)
        mCurrentFrame = Frame(
            mImGray,
            imGrayRight,
            timestamp,
            mpORBextractorLeft,
            mpORBextractorRight,
            mpORBVocabulary,
            mK,
            mDistCoef,
            mbf,
            mThDepth,
            mpCamera,
            mpCamera2,
            mTlr,
            &mLastFrame,
            *mpImuCalib);

    // cout << "Incoming frame ended" << endl;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
    vdStereoMatch_ms.push_back(mCurrentFrame.mTimeStereoMatch);
#endif

    // cout << "Tracking start" << endl;
    Track();
    // cout << "Tracking end" << endl;

    if (mSensor == System::IMU_STEREO)
        return mCurrentFrame.GetImuPose();
    else
        return mCurrentFrame.GetPose().inverse();
}

Sophus::SE3f Tracking::GrabImageMulti(
    const cv::Mat &imLeft,
    const cv::Mat &imRight,
    const cv::Mat &imSideLeft,
    const cv::Mat &imSideRight,
    const cv::Mat &imLeft2,
    const cv::Mat &imRight2,
    const cv::Mat &imSideLeft2,
    const cv::Mat &imSideRight2,
    const cv::Mat &depthLeft2,
    const cv::Mat &depthRight2,
    const cv::Mat &depthSideLeft2,
    const cv::Mat &depthSideRight2,
    const double &timestamp,
    string filename) {

    mImLeft = imLeft;
    mImRight = imRight;
    mImSideLeft = imSideLeft;
    mImSideRight = imSideRight;
    mImLeft2 = imLeft2;
    mImRight2 = imRight2;
    mImSideLeft2 = imSideLeft2;
    mImSideRight2 = imSideRight2;

    mCurrentFrame = Frame(
        imLeft,
        imRight,
        imSideLeft,
        imSideRight,
        imLeft2,
        imRight2,
        imSideLeft2,
        imSideRight2,
        depthLeft2,
        depthRight2,
        depthSideLeft2,
        depthSideRight2,
        timestamp,
        mpORBextractorLeft,
        mpORBextractorRight,
        mpORBextractorSideLeft,
        mpORBextractorSideRight,
        mpORBextractorLeft2,
        mpORBextractorRight2,
        mpORBextractorSideLeft2,
        mpORBextractorSideRight2,
        mbleft,
        mbright,
        mbsideleft,
        mbsideright,
        mbleft2,
        mbright2,
        mbsideleft2,
        mbsideright2,
        mpORBVocabulary,
        mK,
        mDistCoef,
        mbf,
        mThDepth,
        mpCamera,
        mpCamera2,
        mpCamera3,
        mpCamera4,
        mpCamera5,
        mpCamera6,
        mpCamera7,
        mpCamera8,
        mTlr,
        mTlsl,
        mTlsr,
        &mLastFrame,
        *mpImuCalib);

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

    Track();

    if (mSensor == System::IMU_MULTI)
        return mCurrentFrame.GetImuPose();
    else
        return mCurrentFrame.GetPose().inverse();
}

Sophus::SE3f Tracking::GrabImageRGBD(
    const cv::Mat &imRGB,
    const cv::Mat &imD,
    const double &timestamp,
    string filename) {
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if (mImGray.channels() == 3) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
    } else if (mImGray.channels() == 4) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
    }

    if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
        imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

    if (mSensor == System::RGBD)
        mCurrentFrame = Frame(
            mImGray,
            imDepth,
            timestamp,
            mpORBextractorLeft,
            mpORBVocabulary,
            mK,
            mDistCoef,
            mbf,
            mThDepth,
            mpCamera);
    else if (mSensor == System::IMU_RGBD)
        mCurrentFrame = Frame(
            mImGray,
            imDepth,
            timestamp,
            mpORBextractorLeft,
            mpORBVocabulary,
            mK,
            mDistCoef,
            mbf,
            mThDepth,
            mpCamera,
            &mLastFrame,
            *mpImuCalib);

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

    Track();

    return mCurrentFrame.GetPose();
}

Sophus::SE3f
Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename) {
    mImGray = im;
    if (mImGray.channels() == 3) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
    } else if (mImGray.channels() == 4) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
    }

    if (mSensor == System::MONOCULAR) {
        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET || (lastID - initID) < mMaxFrames)
            mCurrentFrame = Frame(
                mImGray,
                timestamp,
                mpIniORBextractor,
                mpORBVocabulary,
                mpCamera,
                mDistCoef,
                mbf,
                mThDepth);
        else
            mCurrentFrame = Frame(
                mImGray,
                timestamp,
                mpORBextractorLeft,
                mpORBVocabulary,
                mpCamera,
                mDistCoef,
                mbf,
                mThDepth);
    } else if (mSensor == System::IMU_MONOCULAR) {
        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET) {
            mCurrentFrame = Frame(
                mImGray,
                timestamp,
                mpIniORBextractor,
                mpORBVocabulary,
                mpCamera,
                mDistCoef,
                mbf,
                mThDepth,
                &mLastFrame,
                *mpImuCalib);
        } else
            mCurrentFrame = Frame(
                mImGray,
                timestamp,
                mpORBextractorLeft,
                mpORBVocabulary,
                mpCamera,
                mDistCoef,
                mbf,
                mThDepth,
                &mLastFrame,
                *mpImuCalib);
    }

    if (mState == NO_IMAGES_YET)
        t0 = timestamp;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

    lastID = mCurrentFrame.mnId;
    Track();

    if (mSensor == System::IMU_MONOCULAR)
        return mCurrentFrame.GetImuPose();
    else
        return mCurrentFrame.GetPose().inverse();
}

void Tracking::GrabImuData(const IMU::Point &imuMeasurement) {
    unique_lock<mutex> lock(mMutexImuQueue);
    mlQueueImuData.push_back(imuMeasurement);
}

void Tracking::PreintegrateIMU() {

    if (!mCurrentFrame.mpPrevFrame) {
        Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    mvImuFromLastFrame.clear();
    mvImuFromLastFrame.reserve(mlQueueImuData.size());
    if (mlQueueImuData.size() == 0) {
        Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    while (true) {
        bool bSleep = false;
        {
            unique_lock<mutex> lock(mMutexImuQueue);
            if (!mlQueueImuData.empty()) {
                IMU::Point *m = &mlQueueImuData.front();
                cout.precision(17);
                if (m->t < mCurrentFrame.mpPrevFrame->mTimeStamp - mImuPer) {
                    mlQueueImuData.pop_front();
                } else if (m->t < mCurrentFrame.mTimeStamp - mImuPer) {
                    mvImuFromLastFrame.push_back(*m);
                    mlQueueImuData.pop_front();
                } else {
                    mvImuFromLastFrame.push_back(*m);
                    break;
                }
            } else {
                break;
                bSleep = true;
            }
        }
        if (bSleep)
            usleep(500);
    }

    const int n = mvImuFromLastFrame.size() - 1;
    if (n == 0) {
        cout << "Empty IMU measurements vector!!!\n";
        return;
    }

    IMU::Preintegrated *pImuPreintegratedFromLastFrame =
        new IMU::Preintegrated(mLastFrame.mImuBias, mCurrentFrame.mImuCalib);

    for (int i = 0; i < n; i++) {
        float tstep;
        Eigen::Vector3f acc, angVel;
        if ((i == 0) && (i < (n - 1))) {
            float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
            float tini = mvImuFromLastFrame[i].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
            acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
                   (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) * (tini / tab)) *
                  0.5f;
            angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                      (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) * (tini / tab)) *
                     0.5f;
            tstep = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
        } else if (i < (n - 1)) {
            acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a) * 0.5f;
            angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w) * 0.5f;
            tstep = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
        } else if ((i > 0) && (i == (n - 1))) {
            float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
            float tend = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mTimeStamp;
            acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
                   (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) * (tend / tab)) *
                  0.5f;
            angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                      (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) * (tend / tab)) *
                     0.5f;
            tstep = mCurrentFrame.mTimeStamp - mvImuFromLastFrame[i].t;
        } else if ((i == 0) && (i == (n - 1))) {
            acc = mvImuFromLastFrame[i].a;
            angVel = mvImuFromLastFrame[i].w;
            tstep = mCurrentFrame.mTimeStamp - mCurrentFrame.mpPrevFrame->mTimeStamp;
        }

        if (!mpImuPreintegratedFromLastKF)
            cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel, tstep);
        pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel, tstep);
    }

    mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

    mCurrentFrame.setIntegrated();

    // Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG);
}

bool Tracking::PredictStateIMU() {
    if (!mCurrentFrame.mpPrevFrame) {
        Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    if (mbMapUpdated && mpLastKeyFrame) {
        const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mpImuPreintegratedFromLastKF->dT;

        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
            Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
        Eigen::Vector3f twb2 =
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz +
            Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
        mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

        mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    } else if (!mbMapUpdated) {
        const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
            Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias));
        Eigen::Vector3f twb2 =
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz +
            Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

        mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

        mCurrentFrame.mImuBias = mLastFrame.mImuBias;
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    } else
        cout << "not IMU prediction!!" << endl;

    return false;
}

void Tracking::ResetFrameIMU() {
    // TODO To implement...
}

void Tracking::Track() {

    if (bStepByStep) {
        std::cout << "Tracking: Waiting to the next step" << std::endl;
        while (!mbStep && bStepByStep)
            usleep(500);
        mbStep = false;
    }

    if (mpLocalMapper->mbBadImu) {
        cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
        mpSystem->ResetActiveMap();
        return;
    }

    Map *pCurrentMap = mpAtlas->GetCurrentMap();
    if (!pCurrentMap) {
        cout << "ERROR: There is not an active map in the atlas" << endl;
    }

    if (mState != NO_IMAGES_YET) {
        if (mLastFrame.mTimeStamp > mCurrentFrame.mTimeStamp) {
            cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
            unique_lock<mutex> lock(mMutexImuQueue);
            mlQueueImuData.clear();
            CreateMapInAtlas();
            return;
        } else if (mCurrentFrame.mTimeStamp > mLastFrame.mTimeStamp + 1.0) {
            // cout << mCurrentFrame.mTimeStamp << ", " << mLastFrame.mTimeStamp << endl;
            // cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId <<
            // endl;
            if (mpAtlas->isInertial()) {

                if (mpAtlas->isImuInitialized()) {
                    cout << "Timestamp jump detected. State set to LOST. Reseting IMU "
                            "integration..."
                         << endl;
                    if (!pCurrentMap->GetIniertialBA2()) {
                        mpSystem->ResetActiveMap();
                    } else {
                        CreateMapInAtlas();
                    }
                } else {
                    cout << "Timestamp jump detected, before IMU initialization. Reseting..."
                         << endl;
                    mpSystem->ResetActiveMap();
                }
                return;
            }
        }
    }

    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
         mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI) &&
        mpLastKeyFrame)
        mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

    if (mState == NO_IMAGES_YET) {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState = mState;

    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
         mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI) &&
        !mbCreatedMap) {
        PreintegrateIMU();
    }
    mbCreatedMap = false;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    mbMapUpdated = false;

    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    int nMapChangeIndex = pCurrentMap->GetLastMapChange();
    if (nCurMapChangeIndex > nMapChangeIndex) {
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;
    }

    if (mState == NOT_INITIALIZED) {
        if (mSensor == System::STEREO || mSensor == System::RGBD ||
            mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            StereoInitialization();
        else if (mSensor == System::IMU_MULTI)
            MultiInitialization();
        else
            MonocularInitialization();

        if (mState != OK) // If rightly initialized, mState=OK
        {
            mLastFrame = Frame(mCurrentFrame);
            return;
        }

        if (mpAtlas->GetAllMaps().size() == 1) {
            mnFirstFrameId = mCurrentFrame.mnId;
        }
    } else {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is
        // lost)
        if (!mbOnlyTracking) {
            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            if (mState == OK) {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if ((!mbVelocity && !pCurrentMap->isImuInitialized()) ||
                    mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
                    Verbose::PrintMess(
                        "TRACK: Track with respect to the reference KF ",
                        Verbose::VERBOSITY_DEBUG);
                    bOK = TrackReferenceKeyFrame();
                } else {
                    Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackWithMotionModel();
                    if (!bOK) {
                        bOK = TrackReferenceKeyFrame();
                    }
                }

                if (!bOK) {
                    if (mCurrentFrame.mnId <= (mnLastRelocFrameId + mnFramesToResetIMU) &&
                        (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
                         mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI)) {
                        mState = LOST;
                    } else if (pCurrentMap->KeyFramesInMap() > 10) {
                        // cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                        mState = RECENTLY_LOST;
                        mTimeStampLost = mCurrentFrame.mTimeStamp;
                    } else {
                        mState = LOST;
                    }
                }
            } else {
                std::cout << "mState!=OK" << std::endl;

                if (mState == RECENTLY_LOST) {
                    Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);

                    bOK = true;
                    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
                        mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI) {
                        if (pCurrentMap->isImuInitialized())
                            PredictStateIMU();
                        else
                            bOK = false;

                        if (mCurrentFrame.mTimeStamp - mTimeStampLost > time_recently_lost) {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK = false;
                        }
                    } else {
                        // Relocalization
                        bOK = Relocalization();
                        // std::cout << "mCurrentFrame.mTimeStamp:" <<
                        // to_string(mCurrentFrame.mTimeStamp) << std::endl; std::cout <<
                        // "mTimeStampLost:" << to_string(mTimeStampLost) << std::endl;
                        if (mCurrentFrame.mTimeStamp - mTimeStampLost > 3.0f && !bOK) {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK = false;
                        }
                    }
                } else if (mState == LOST) {

                    Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);

                    if (pCurrentMap->KeyFramesInMap() < 10) {
                        mpSystem->ResetActiveMap();
                        Verbose::PrintMess("Reseting current map...", Verbose::VERBOSITY_NORMAL);
                    } else
                        CreateMapInAtlas();

                    if (mpLastKeyFrame)
                        mpLastKeyFrame = static_cast<KeyFrame *>(NULL);

                    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

                    return;
                }
            }

        } else {
            // Localization Mode: Local Mapping is deactivated (TODO Not available in inertial
            // mode)
            if (mState == LOST) {
                if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
                    mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI)
                    Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                bOK = Relocalization();
            } else {
                if (!mbVO) {
                    // In last frame we tracked enough MapPoints in the map
                    if (mbVelocity) {
                        bOK = TrackWithMotionModel();
                    } else {
                        bOK = TrackReferenceKeyFrame();
                    }
                } else {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing
                    // relocalization. If relocalization is sucessfull we choose that solution,
                    // otherwise we retain the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint *> vpMPsMM;
                    vector<bool> vbOutMM;
                    Sophus::SE3f TcwMM;
                    if (mbVelocity) {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.GetPose();
                    }
                    bOKReloc = Relocalization();

                    if (bOKMM && !bOKReloc) {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if (mbVO) {
                            for (int i = 0; i < mCurrentFrame.N; i++) {
                                if (mCurrentFrame.mvpMapPoints[i] &&
                                    !mCurrentFrame.mvbOutlier[i]) {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    } else if (bOKReloc) {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        if (!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if (!mbOnlyTracking) {
            if (bOK) {
                bOK = TrackLocalMap();
            }
            if (!bOK)
                cout << "Fail to track local map!" << endl;
        } else {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot
            // retrieve a local map and therefore we do not perform TrackLocalMap(). Once the
            // system relocalizes the camera we will use the local map again.
            if (bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if (bOK)
            mState = OK;
        else if (mState == OK) {
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
                mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI) {
                Verbose::PrintMess(
                    "Track lost for less than one second...",
                    Verbose::VERBOSITY_NORMAL);
                // if(!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2())
                // {
                //     cout << "IMU is not or recently initialized. Reseting active map..." <<
                //     endl; mpSystem->ResetActiveMap();
                // }

                mState = RECENTLY_LOST;
            } else
                mState = RECENTLY_LOST; // visual to lost

            /*if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
            {*/
            mTimeStampLost = mCurrentFrame.mTimeStamp;
            //}
        }

        // Save frame if recent relocalization, since they are used for IMU reset (as we are making
        // copy, it shluld be once mCurrFrame is completely modified)
        if ((mCurrentFrame.mnId < (mnLastRelocFrameId + mnFramesToResetIMU)) &&
            (mCurrentFrame.mnId > mnFramesToResetIMU) &&
            (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
             mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI) &&
            pCurrentMap->isImuInitialized()) {
            // TODO check this situation
            Verbose::PrintMess(
                "Saving pointer to frame. imu needs reset...",
                Verbose::VERBOSITY_NORMAL);
            Frame *pF = new Frame(mCurrentFrame);
            pF->mpPrevFrame = new Frame(mLastFrame);

            // Load preintegration
            pF->mpImuPreintegratedFrame =
                new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
        }

        if (pCurrentMap->isImuInitialized()) {
            if (bOK) {
                if (mCurrentFrame.mnId == (mnLastRelocFrameId + mnFramesToResetIMU)) {
                    cout << "RESETING FRAME!!!" << endl;
                    ResetFrameIMU();
                } else if (mCurrentFrame.mnId > (mnLastRelocFrameId + 30))
                    mLastBias = mCurrentFrame.mImuBias;
            }
        }

        // Update drawer
        mpFrameDrawer->Update(this);
        if (pCurrentMap->isImuInitialized())
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        if (bOK || mState == RECENTLY_LOST) {
            // Update motion model
            if (mLastFrame.isSet() && mCurrentFrame.isSet()) {
                Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();
                mVelocity = mCurrentFrame.GetPose() * LastTwc;
                mbVelocity = true;
            } else {
                mbVelocity = false;
            }

            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
                mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI)
                if (pCurrentMap->isImuInitialized())
                    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

            // Clean VO matches
            for (int i = 0; i < mCurrentFrame.N; i++) {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (pMP)
                    if (pMP->Observations() < 1) {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(),
                                            lend = mlpTemporalPoints.end();
                 lit != lend;
                 lit++) {
                MapPoint *pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartNewKF =
                std::chrono::steady_clock::now();
#endif
            bool bNeedKF = NeedNewKeyFrame();

            // Check if we need to insert a new keyframe
            // if(bNeedKF && bOK)
            if (bNeedKF &&
                (bOK || (mInsertKFsLost && mState == RECENTLY_LOST &&
                         (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
                          mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI))))
                CreateNewKeyFrame();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndNewKF = std::chrono::steady_clock::now();

            double timeNewKF =
                std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                    time_EndNewKF - time_StartNewKF)
                    .count();
            vdNewKF_ms.push_back(timeNewKF);
#endif

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is
            // tracked
            for (int i = 0; i < mCurrentFrame.N; i++) {
                if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if (mState == LOST) {
            if (pCurrentMap->KeyFramesInMap() <= 10) {
                mpSystem->ResetActiveMap();
                return;
            }
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
                mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI)
                if (!pCurrentMap->isImuInitialized()) {
                    Verbose::PrintMess(
                        "Track lost before IMU initialisation, reseting...",
                        Verbose::VERBOSITY_QUIET);
                    mpSystem->ResetActiveMap();
                    return;
                }

            CreateMapInAtlas();

            return;
        }

        if (!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    if (mState == OK || mState == RECENTLY_LOST) {
        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        if (mCurrentFrame.isSet()) {
            Sophus::SE3f Tcr_ =
                mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr_);
            mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState == LOST);
        } else {
            // This can happen if tracking is lost
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState == LOST);
        }
    }

#ifdef REGISTER_LOOP
    if (Stop()) {

        // Safe area to stop
        while (isStopped()) {
            usleep(3000);
        }
    }
#endif
}

void Tracking::StereoInitialization() {
    if (mCurrentFrame.N > 500) {
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
            if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated) {
                cout << "not IMU meas" << endl;
                return;
            }

            if (!mFastInit && (mCurrentFrame.mpImuPreintegratedFrame->avgA -
                               mLastFrame.mpImuPreintegratedFrame->avgA)
                                      .norm() < 0.5) {
                cout << "not enough acceleration" << endl;
                return;
            }

            if (mpImuPreintegratedFromLastKF)
                delete mpImuPreintegratedFromLastKF;

            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
            mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin (In case of inertial SLAM to imu)
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
            Eigen::Matrix3f Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix();
            Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();
            Eigen::Vector3f Vwb0;
            Vwb0.setZero();
            mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);
        } else
            mCurrentFrame.SetPose(Sophus::SE3f());

        // Create KeyFrame
        KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpAtlas->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        if (!mpCamera2) {
            for (int i = 0; i < mCurrentFrame.N; i++) {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0) {
                    Eigen::Vector3f x3D;
                    mCurrentFrame.UnprojectStereo(i, x3D);
                    MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKFini, i);
                    pKFini->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                }
            }
        } else {
            for (int i = 0; i < mCurrentFrame.Nleft; i++) {
                int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
                if (rightIndex != -1) {
                    Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

                    MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

                    pNewMP->AddObservation(pKFini, i);
                    pNewMP->AddObservation(pKFini, rightIndex + mCurrentFrame.Nleft);

                    pKFini->AddMapPoint(pNewMP, i);
                    pKFini->AddMapPoint(pNewMP, rightIndex + mCurrentFrame.Nleft);

                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                    mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft] = pNewMP;
                }
            }
        }

        Verbose::PrintMess(
            "New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points",
            Verbose::VERBOSITY_QUIET);

        // cout << "Active map: " << mpAtlas->GetCurrentMap()->GetId() << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;
        // mnLastRelocFrameId = mCurrentFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        mState = OK;
    }
}

void Tracking::MultiInitialization() {
    if (mCurrentFrame.N > 500) {
        if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated) {
            cout << "not IMU meas" << endl;
            return;
        }

        if (mpImuPreintegratedFromLastKF)
            delete mpImuPreintegratedFromLastKF;

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
        mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        Eigen::Matrix3f Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix();
        Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();
        Eigen::Vector3f Vwb0;
        Vwb0.setZero();
        mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);

        // Create KeyFrame
        KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpAtlas->AddKeyFrame(pKFini);

        for (int i = 0; i < mCurrentFrame.Nleft; i++) {
            int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
            if (rightIndex != -1) {
                Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

                MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

                pNewMP->AddObservation(pKFini, i);
                pNewMP->AddObservation(pKFini, rightIndex + mCurrentFrame.Nleft);

                pKFini->AddMapPoint(pNewMP, i);
                pKFini->AddMapPoint(pNewMP, rightIndex + mCurrentFrame.Nleft);

                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpAtlas->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i] = pNewMP;
                mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft] = pNewMP;
            }
        }

        Verbose::PrintMess(
            "New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points",
            Verbose::VERBOSITY_QUIET);

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        mState = OK;
    }
}

void Tracking::MonocularInitialization() {

    if (!mbReadyToInitializate) {
        // Set Reference Frame
        if (mCurrentFrame.mvKeys.size() > 100) {

            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            if (mSensor == System::IMU_MONOCULAR) {
                if (mpImuPreintegratedFromLastKF) {
                    delete mpImuPreintegratedFromLastKF;
                }
                mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
            }

            mbReadyToInitializate = true;

            return;
        }
    } else {
        if (((int)mCurrentFrame.mvKeys.size() <= 100) ||
            ((mSensor == System::IMU_MONOCULAR) &&
             (mLastFrame.mTimeStamp - mInitialFrame.mTimeStamp > 1.0))) {
            mbReadyToInitializate = false;

            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9, true);
        int nmatches = matcher.SearchForInitialization(
            mInitialFrame,
            mCurrentFrame,
            mvbPrevMatched,
            mvIniMatches,
            100);

        // Check if there are enough correspondences
        if (nmatches < 100) {
            mbReadyToInitializate = false;
            return;
        }

        Sophus::SE3f Tcw;
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if (mpCamera->ReconstructWithTwoViews(
                mInitialFrame.mvKeysUn,
                mCurrentFrame.mvKeysUn,
                mvIniMatches,
                Tcw,
                mvIniP3D,
                vbTriangulated)) {
            for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
                if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
                    mvIniMatches[i] = -1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(Sophus::SE3f());
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular() {
    // Create KeyFrames
    KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);
    KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

    if (mSensor == System::IMU_MONOCULAR)
        pKFini->mpImuPreintegrated = (IMU::Preintegrated *)(NULL);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpAtlas->AddKeyFrame(pKFini);
    mpAtlas->AddKeyFrame(pKFcur);

    for (size_t i = 0; i < mvIniMatches.size(); i++) {
        if (mvIniMatches[i] < 0)
            continue;

        // Create MapPoint.
        Eigen::Vector3f worldPos;
        worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
        MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpAtlas->GetCurrentMap());

        pKFini->AddMapPoint(pMP, i);
        pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

        pMP->AddObservation(pKFini, i);
        pMP->AddObservation(pKFcur, mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        // Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        // Add to Map
        mpAtlas->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    std::set<MapPoint *> sMPs;
    sMPs = pKFini->GetMapPoints();

    // Bundle Adjustment
    Verbose::PrintMess(
        "New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points",
        Verbose::VERBOSITY_QUIET);
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(), 20);

    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth;
    if (mSensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f / medianDepth; // 4.0f
    else
        invMedianDepth = 1.0f / medianDepth;

    if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 50) // TODO Check, originally 100 tracks
    {
        Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_QUIET);
        mpSystem->ResetActiveMap();
        return;
    }

    // Scale initial baseline
    Sophus::SE3f Tc2w = pKFcur->GetPose();
    Tc2w.translation() *= invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
        if (vpAllMapPoints[iMP]) {
            MapPoint *pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    if (mSensor == System::IMU_MONOCULAR) {
        pKFcur->mPrevKF = pKFini;
        pKFini->mNextKF = pKFcur;
        pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(
            pKFcur->mpImuPreintegrated->GetUpdatedBias(),
            pKFcur->mImuCalib);
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    mpLocalMapper->mFirstTs = pKFcur->mTimeStamp;

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;
    // mnLastRelocFrameId = mInitialFrame.mnId;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    // Compute here initial velocity
    vector<KeyFrame *> vKFs = mpAtlas->GetAllKeyFrames();

    Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
    mbVelocity = false;
    Eigen::Vector3f phi = deltaT.so3().log();

    double aux = (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp) /
                 (mCurrentFrame.mTimeStamp - mInitialFrame.mTimeStamp);
    phi *= aux;

    mLastFrame = Frame(mCurrentFrame);

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    mState = OK;

    initID = pKFcur->mnId;
}

void Tracking::CreateMapInAtlas() {
    mnLastInitFrameId = mCurrentFrame.mnId;
    mpAtlas->CreateNewMap();
    if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR ||
        mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI)
        mpAtlas->SetInertialSensor();
    mbSetInit = false;

    mnInitialFrameId = mCurrentFrame.mnId + 1;
    mState = NO_IMAGES_YET;

    // Restart the variable with information about the last KF
    mbVelocity = false;
    // mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the current id,
    // because it is the new starting point for new map
    Verbose::PrintMess(
        "First frame id in map: " + to_string(mnLastInitFrameId + 1),
        Verbose::VERBOSITY_NORMAL);
    mbVO = false; // Init value for know if there are enough MapPoints in the last KF
    if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR) {
        mbReadyToInitializate = false;
    }

    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
         mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI) &&
        mpImuPreintegratedFromLastKF) {
        delete mpImuPreintegratedFromLastKF;
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
    }

    if (mpLastKeyFrame)
        mpLastKeyFrame = static_cast<KeyFrame *>(NULL);

    if (mpReferenceKF)
        mpReferenceKF = static_cast<KeyFrame *>(NULL);

    mLastFrame = Frame();
    mCurrentFrame = Frame();
    mvIniMatches.clear();

    mbCreatedMap = true;
}

void Tracking::CheckReplacedInLastFrame() {
    for (int i = 0; i < mLastFrame.N; i++) {
        MapPoint *pMP = mLastFrame.mvpMapPoints[i];

        if (pMP) {
            MapPoint *pRep = pMP->GetReplaced();
            if (pRep) {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

bool Tracking::TrackReferenceKeyFrame() {
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7, true);
    vector<MapPoint *> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

    if (nmatches < 15) {
        cout << "TRACK_REF_KF: Less than 15 matches!!\n";
        return false;
    }

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.GetPose());

    // mCurrentFrame.PrintPointDistribution();

    // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        // if(i >= mCurrentFrame.Nleft) break;
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (mCurrentFrame.mvbOutlier[i]) {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                mCurrentFrame.mvbOutlier[i] = false;
                if (i < mCurrentFrame.Nleft)
                    pMP->mbTrackInView = false;
                else if (
                    i < mCurrentFrame.Nleft + mCurrentFrame.Nright && i >= mCurrentFrame.Nleft)
                    pMP->mbTrackInViewR = false;
                else if (
                    i < mCurrentFrame.Nleft + mCurrentFrame.Nright + mCurrentFrame.Nsideleft &&
                    i >= mCurrentFrame.Nleft + mCurrentFrame.Nright)
                    pMP->mbTrackInViewSL = false;
                else
                    pMP->mbTrackInViewSR = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
        mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI)
        return true;
    else
        return nmatchesMap >= 10;
}

void Tracking::UpdateLastFrame() {
    // Update pose according to reference keyframe
    KeyFrame *pRef = mLastFrame.mpReferenceKF;
    Sophus::SE3f Tlr = mlRelativeFramePoses.back();
    mLastFrame.SetPose(Tlr * pRef->GetPose());

    if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR ||
        mSensor == System::IMU_MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float, int>> vDepthIdx;
    const int Nfeat = mLastFrame.Nleft == -1 ? mLastFrame.N : mLastFrame.Nleft;
    vDepthIdx.reserve(Nfeat);
    for (int i = 0; i < Nfeat; i++) {
        float z = mLastFrame.mvDepth[i];
        if (z > 0) {
            vDepthIdx.push_back(make_pair(z, i));
        }
    }

    if (vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(), vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for (size_t j = 0; j < vDepthIdx.size(); j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint *pMP = mLastFrame.mvpMapPoints[i];

        if (!pMP)
            bCreateNew = true;
        else if (pMP->Observations() < 1)
            bCreateNew = true;

        if (bCreateNew) {
            Eigen::Vector3f x3D;

            if (mLastFrame.Nleft == -1) {
                mLastFrame.UnprojectStereo(i, x3D);
            } else {
                x3D = mLastFrame.UnprojectStereoFishEye(i);
            }

            MapPoint *pNewMP = new MapPoint(x3D, mpAtlas->GetCurrentMap(), &mLastFrame, i);
            mLastFrame.mvpMapPoints[i] = pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        } else {
            nPoints++;
        }

        if (vDepthIdx[j].first > mThDepth && nPoints > 100)
            break;
    }
}

bool Tracking::TrackWithMotionModel() {
    ORBmatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    if (mpAtlas->isImuInitialized() &&
        (mCurrentFrame.mnId > mnLastRelocFrameId + mnFramesToResetIMU)) {
        // Predict state with IMU if it is initialized and it doesnt need reset
        PredictStateIMU();
        return true;
    } else {
        mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
    }

    fill(
        mCurrentFrame.mvpMapPoints.begin(),
        mCurrentFrame.mvpMapPoints.end(),
        static_cast<MapPoint *>(NULL));

    // Project points seen in previous frame
    int th;

    if (mSensor == System::STEREO)
        th = 7;
    else
        th = 15;

    int nmatches = matcher.SearchByProjection(
        mCurrentFrame,
        mLastFrame,
        th,
        mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);

    // If few matches, uses a wider window search
    if (nmatches < 20) {
        Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
        fill(
            mCurrentFrame.mvpMapPoints.begin(),
            mCurrentFrame.mvpMapPoints.end(),
            static_cast<MapPoint *>(NULL));

        nmatches = matcher.SearchByProjection(
            mCurrentFrame,
            mLastFrame,
            2 * th,
            mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);
        Verbose::PrintMess(
            "Matches with wider search: " + to_string(nmatches),
            Verbose::VERBOSITY_NORMAL);
    }

    if (nmatches < 20) {
        Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
            mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI)
            return true;
        else
            return false;
    }

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (mCurrentFrame.mvbOutlier[i]) {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                mCurrentFrame.mvbOutlier[i] = false;
                if (i < mCurrentFrame.Nleft)
                    pMP->mbTrackInView = false;
                else if (
                    i < mCurrentFrame.Nleft + mCurrentFrame.Nright && i >= mCurrentFrame.Nleft)
                    pMP->mbTrackInViewR = false;
                else if (
                    i < mCurrentFrame.Nleft + mCurrentFrame.Nright + mCurrentFrame.Nsideleft &&
                    i >= mCurrentFrame.Nleft + mCurrentFrame.Nright)
                    pMP->mbTrackInViewSL = false;
                else
                    pMP->mbTrackInViewSR = false;

                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }

    if (mbOnlyTracking) {
        mbVO = nmatchesMap < 10;
        return nmatches > 20;
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
        mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI)
        return true;
    else
        return nmatchesMap >= 10;
}

bool Tracking::TrackLocalMap() {

    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    mTrackedFr++;

    UpdateLocalMap();
    SearchLocalPoints();

    // TODO check outliers before PO
    int aux1 = 0, aux2 = 0;
    for (int i = 0; i < mCurrentFrame.N; i++)
        if (mCurrentFrame.mvpMapPoints[i]) {
            aux1++;
            if (mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    int inliers;
    if (!mpAtlas->isImuInitialized()) {
        Optimizer::PoseOptimization(&mCurrentFrame);
    } else {
        if (mCurrentFrame.mnId <= mnLastRelocFrameId + mnFramesToResetIMU) {
            Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
            Optimizer::PoseOptimization(&mCurrentFrame);
        } else {
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            if (!mbMapUpdated) //  && (mnMatchesInliers>30))
            {
                Verbose::PrintMess(
                    "TLM: PoseInertialOptimizationLastFrame ",
                    Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastFrame(
                    &mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            } else {
                Verbose::PrintMess(
                    "TLM: PoseInertialOptimizationLastKeyFrame ",
                    Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(
                    &mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
        }
    }

    aux1 = 0, aux2 = 0;
    for (int i = 0; i < mCurrentFrame.N; i++)
        if (mCurrentFrame.mvpMapPoints[i]) {
            aux1++;
            if (mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (!mCurrentFrame.mvbOutlier[i]) {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if (!mbOnlyTracking) {
                    if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                        mnMatchesInliers++;
                } else
                    mnMatchesInliers++;
            } else if (mSensor == System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    mpLocalMapper->mnMatchesInliers = mnMatchesInliers;
    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
        return false;

    if ((mnMatchesInliers > 10) && (mState == RECENTLY_LOST))
        return true;

    if (mSensor == System::IMU_MONOCULAR) {
        if ((mnMatchesInliers < 15 && mpAtlas->isImuInitialized()) ||
            (mnMatchesInliers < 50 && !mpAtlas->isImuInitialized())) {
            return false;
        } else
            return true;
    } else if (
        mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD ||
        mSensor == System::IMU_MULTI) {
        if (mnMatchesInliers < 15) {
            return false;
        } else
            return true;
    } else {
        if (mnMatchesInliers < 30)
            return false;
        else
            return true;
    }
}

bool Tracking::NeedNewKeyFrame() {
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
         mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI) &&
        !mpAtlas->GetCurrentMap()->isImuInitialized()) {
        if (mSensor == System::IMU_MONOCULAR &&
            (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
            return true;
        else if (
            (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD ||
             mSensor == System::IMU_MULTI) &&
            (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
            return true;
        else
            return false;
    }

    if (mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()) {
        /*if(mSensor == System::MONOCULAR)
        {
            std::cout << "NeedNewKeyFrame: localmap stopped" << std::endl;
        }*/
        return false;
    }

    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames) {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if (nKFs <= 2)
        nMinObs = 2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose = 0;

    if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) {
        int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for (int i = 0; i < N; i++) {
            if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth) {
                if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
        // Verbose::PrintMess("[NEEDNEWKF]-> closed points: " + to_string(nTrackedClose) + "; non
        // tracked closed points: " + to_string(nNonTrackedClose), Verbose::VERBOSITY_NORMAL);//
        // Verbose::VERBOSITY_DEBUG);
    }

    bool bNeedToInsertClose;
    bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

    // Thresholds
    float thRefRatio = 0.75f;
    if (nKFs < 2)
        thRefRatio = 0.4f;

    /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
    const int thStereoClosedPoints = 15;
    if(nClosedPoints < thStereoClosedPoints && (mSensor==System::STEREO ||
    mSensor==System::IMU_STEREO))
    {
        //Pseudo-monocular, there are not enough close points to be confident about the stereo
    observations. thRefRatio = 0.9f;
    }*/

    if (mSensor == System::MONOCULAR)
        thRefRatio = 0.9f;

    if (mpCamera2)
        thRefRatio = 0.75f;

    if (mSensor == System::IMU_MONOCULAR) {
        if (mnMatchesInliers > 350) // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b =
        ((mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames) &&
         bLocalMappingIdle); // mpLocalMapper->KeyframesInQueue() < 2);
    // Condition 1c: tracking is weak
    const bool c1c = mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR &&
                     mSensor != System::IMU_STEREO && mSensor != System::IMU_RGBD &&
                     mSensor != System::IMU_MULTI &&
                     (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry
    // compared to map matches.
    const bool c2 =
        (((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose)) &&
         mnMatchesInliers > 15);

    // std::cout << "NeedNewKF: c1a=" << c1a << "; c1b=" << c1b << "; c1c=" << c1c << "; c2=" << c2
    // << std::endl;
    //  Temporal condition for Inertial cases
    bool c3 = false;
    if (mpLastKeyFrame) {
        if (mSensor == System::IMU_MONOCULAR) {
            if ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
                c3 = true;
        } else if (
            mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD ||
            mSensor == System::IMU_MULTI) {
            if ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
                c3 = true;
        }
    }

    bool c4 = false;
    if ((((mnMatchesInliers < 75) && (mnMatchesInliers > 15)) || mState == RECENTLY_LOST) &&
        (mSensor == System::IMU_MONOCULAR)) // MODIFICATION_2, originally ((((mnMatchesInliers<75)
                                            // && (mnMatchesInliers>15)) || mState==RECENTLY_LOST)
                                            // && ((mSensor == System::IMU_MONOCULAR)))
        c4 = true;
    else
        c4 = false;

    if (((c1a || c1b || c1c) && c2) || c3 || c4) {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if (bLocalMappingIdle || mpLocalMapper->IsInitializing()) {
            return true;
        } else {
            mpLocalMapper->InterruptBA();
            if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) {
                if (mpLocalMapper->KeyframesInQueue() < 3)
                    return true;
                else
                    return false;
            } else {
                // std::cout << "NeedNewKeyFrame: localmap is busy" << std::endl;
                return false;
            }
        }
    } else
        return false;
}

void Tracking::CreateNewKeyFrame() {
    if (mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized())
        return;

    if (!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

    if (mpAtlas->isImuInitialized()) //  || mpLocalMapper->IsInitializing())
        pKF->bImu = true;

    pKF->SetNewBias(mCurrentFrame.mImuBias);
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if (mpLastKeyFrame) {
        pKF->mPrevKF = mpLastKeyFrame;
        mpLastKeyFrame->mNextKF = pKF;
    } else
        Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

    // Reset preintegration from last KF (Create new object)
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
        mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI) {
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(), pKF->mImuCalib);
    }

    if (mSensor != System::MONOCULAR &&
        mSensor != System::IMU_MONOCULAR) // TODO check if incluide imu_stereo
    {
        mCurrentFrame.UpdatePoseMatrices();
        // cout << "create new MPs" << endl;
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        int maxPoint = 100;
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            maxPoint = 100;

        vector<pair<float, int>> vDepthIdx;
        int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
        vDepthIdx.reserve(mCurrentFrame.N);
        for (int i = 0; i < N; i++) {
            float z = mCurrentFrame.mvDepth[i];
            if (z > 0) {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (!vDepthIdx.empty()) {
            sort(vDepthIdx.begin(), vDepthIdx.end());

            int nPoints = 0;
            for (size_t j = 0; j < vDepthIdx.size(); j++) {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (!pMP)
                    bCreateNew = true;
                else if (pMP->Observations() < 1) {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }

                if (bCreateNew) {
                    Eigen::Vector3f x3D;

                    if (mCurrentFrame.Nleft == -1) {
                        mCurrentFrame.UnprojectStereo(i, x3D);
                    } else {
                        x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                    }

                    MapPoint *pNewMP = new MapPoint(x3D, pKF, mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKF, i);

                    // Check if it is a stereo observation in order to not
                    // duplicate mappoints
                    if (mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0) {
                        mCurrentFrame.mvpMapPoints
                            [mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]] = pNewMP;
                        pNewMP->AddObservation(
                            pKF,
                            mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        pKF->AddMapPoint(
                            pNewMP,
                            mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                    }

                    pKF->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                    nPoints++;
                } else {
                    nPoints++;
                }

                if (vDepthIdx[j].first > mThDepth && nPoints > maxPoint) {
                    break;
                }
            }
            // Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints),
            // Verbose::VERBOSITY_NORMAL);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints() {
    // Do not search map points already matched
    for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(),
                                      vend = mCurrentFrame.mvpMapPoints.end();
         vit != vend;
         vit++) {
        MapPoint *pMP = *vit;
        if (pMP) {
            if (pMP->isBad()) {
                *vit = static_cast<MapPoint *>(NULL);
            } else {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
                pMP->mbTrackInViewSL = false;
                pMP->mbTrackInViewSR = false;
            }
        }
    }

    int nToMatch = 0;

    // Project points in frame and check its visibility
    for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(),
                                      vend = mvpLocalMapPoints.end();
         vit != vend;
         vit++) {
        MapPoint *pMP = *vit;

        if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if (pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
            pMP->IncreaseVisible();
            nToMatch++;
        }
        if (pMP->mbTrackInView) {
            mCurrentFrame.mmProjectPoints[pMP->mnId] =
                cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
    }

    if (nToMatch > 0) {
        ORBmatcher matcher(0.8);
        int th = 1;
        if (mSensor == System::RGBD || mSensor == System::IMU_RGBD)
            th = 3;
        if (mpAtlas->isImuInitialized()) {
            if (mpAtlas->GetCurrentMap()->GetIniertialBA2())
                th = 2;
            else
                th = 6;
        } else if (
            !mpAtlas->isImuInitialized() &&
            (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
             mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI)) {
            th = 10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
            th = 5;

        if (mState == LOST || mState == RECENTLY_LOST) // Lost for less than 1 second
            th = 15;                                   // 15

        int matches = matcher.SearchByProjection(
            mCurrentFrame,
            mvpLocalMapPoints,
            th,
            mpLocalMapper->mbFarPoints,
            mpLocalMapper->mThFarPoints);
    }
}

void Tracking::UpdateLocalMap() {
    // This is for visualization
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints() {
    mvpLocalMapPoints.clear();

    int count_pts = 0;

    for (vector<KeyFrame *>::const_reverse_iterator itKF = mvpLocalKeyFrames.rbegin(),
                                                    itEndKF = mvpLocalKeyFrames.rend();
         itKF != itEndKF;
         ++itKF) {
        KeyFrame *pKF = *itKF;
        const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

        for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end();
             itMP != itEndMP;
             itMP++) {

            MapPoint *pMP = *itMP;
            if (!pMP)
                continue;
            if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                continue;
            if (!pMP->isBad()) {
                count_pts++;
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
            }
        }
    }
}

void Tracking::UpdateLocalKeyFrames() {
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame *, int> keyframeCounter;
    if (!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId < mnLastRelocFrameId + 2)) {
        for (int i = 0; i < mCurrentFrame.N; i++) {
            MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
            if (pMP) {
                if (!pMP->isBad()) {
                    const map<KeyFrame *, tuple<int, int, int, int>> observations =
                        pMP->GetObservations();
                    for (map<KeyFrame *, tuple<int, int, int, int>>::const_iterator
                             it = observations.begin(),
                             itend = observations.end();
                         it != itend;
                         it++)
                        keyframeCounter[it->first]++;
                } else {
                    mCurrentFrame.mvpMapPoints[i] = NULL;
                }
            }
        }
    } else {
        for (int i = 0; i < mLastFrame.N; i++) {
            // Using lastframe since current frame has not matches yet
            if (mLastFrame.mvpMapPoints[i]) {
                MapPoint *pMP = mLastFrame.mvpMapPoints[i];
                if (!pMP)
                    continue;
                if (!pMP->isBad()) {
                    const map<KeyFrame *, tuple<int, int, int, int>> observations =
                        pMP->GetObservations();
                    for (map<KeyFrame *, tuple<int, int, int, int>>::const_iterator
                             it = observations.begin(),
                             itend = observations.end();
                         it != itend;
                         it++)
                        keyframeCounter[it->first]++;
                } else {
                    // MODIFICATION
                    mLastFrame.mvpMapPoints[i] = NULL;
                }
            }
        }
    }

    int max = 0;
    KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which
    // keyframe shares most points
    for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(),
                                              itEnd = keyframeCounter.end();
         it != itEnd;
         it++) {
        KeyFrame *pKF = it->first;

        if (pKF->isBad())
            continue;

        if (it->second > max) {
            max = it->second;
            pKFmax = pKF;
        }

        mvpLocalKeyFrames.push_back(pKF);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to already-included
    // keyframes
    for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(),
                                            itEndKF = mvpLocalKeyFrames.end();
         itKF != itEndKF;
         itKF++) {
        // Limit the number of keyframes
        if (mvpLocalKeyFrames.size() > 80) // 80
            break;

        KeyFrame *pKF = *itKF;

        const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(),
                                                itEndNeighKF = vNeighs.end();
             itNeighKF != itEndNeighKF;
             itNeighKF++) {
            KeyFrame *pNeighKF = *itNeighKF;
            if (!pNeighKF->isBad()) {
                if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame *> spChilds = pKF->GetChilds();
        for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end();
             sit != send;
             sit++) {
            KeyFrame *pChildKF = *sit;
            if (!pChildKF->isBad()) {
                if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame *pParent = pKF->GetParent();
        if (pParent) {
            if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                break;
            }
        }
    }

    // Add 10 last temporal KFs (mainly for IMU)
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
         mSensor == System::IMU_RGBD || mSensor == System::IMU_MULTI) &&
        mvpLocalKeyFrames.size() < 80) {
        KeyFrame *tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

        const int Nd = 20;
        for (int i = 0; i < Nd; i++) {
            if (!tempKeyFrame)
                break;
            if (tempKeyFrame->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                mvpLocalKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                tempKeyFrame = tempKeyFrame->mPrevKF;
            }
        }
    }

    if (pKFmax) {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization() {
    Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame *> vpCandidateKFs =
        mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

    if (vpCandidateKFs.empty()) {
        Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75, true);

    vector<MLPnPsolver *> vpMLPnPsolvers;
    vpMLPnPsolvers.resize(nKFs);

    vector<vector<MapPoint *>> vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

    for (int i = 0; i < nKFs; i++) {
        KeyFrame *pKF = vpCandidateKFs[i];
        if (pKF->isBad())
            vbDiscarded[i] = true;
        else {
            int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
            if (nmatches < 15) {
                vbDiscarded[i] = true;
                continue;
            } else {
                MLPnPsolver *pSolver = new MLPnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(
                    0.99,
                    10,
                    300,
                    6,
                    0.5,
                    5.991); // This solver needs at least 6 points
                vpMLPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9, true);

    while (nCandidates > 0 && !bMatch) {
        for (int i = 0; i < nKFs; i++) {
            if (vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            MLPnPsolver *pSolver = vpMLPnPsolvers[i];
            Eigen::Matrix4f eigTcw;
            bool bTcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers, eigTcw);

            // If Ransac reachs max. iterations discard keyframe
            if (bNoMore) {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if (bTcw) {
                Sophus::SE3f Tcw(eigTcw);
                mCurrentFrame.SetPose(Tcw);
                // Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint *> sFound;

                const int np = vbInliers.size();

                for (int j = 0; j < np; j++) {
                    if (vbInliers[j]) {
                        mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    } else
                        mCurrentFrame.mvpMapPoints[j] = NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if (nGood < 10)
                    continue;

                for (int io = 0; io < mCurrentFrame.N; io++)
                    if (mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if (nGood < 50) {
                    int nadditional =
                        matcher2
                            .SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

                    if (nadditional + nGood >= 50) {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a
                        // narrower window the camera has been already optimized with many points
                        if (nGood > 30 && nGood < 50) {
                            sFound.clear();
                            for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                if (mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional = matcher2.SearchByProjection(
                                mCurrentFrame,
                                vpCandidateKFs[i],
                                sFound,
                                3,
                                64);

                            // Final optimization
                            if (nGood + nadditional >= 50) {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for (int io = 0; io < mCurrentFrame.N; io++)
                                    if (mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io] = NULL;
                            }
                        }
                    }
                }

                // If the pose is supported by enough inliers stop ransacs and continue
                if (nGood >= 50) {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if (!bMatch) {
        return false;
    } else {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        cout << "Relocalized!!" << endl;
        return true;
    }
}

void Tracking::Reset(bool bLocMap) {
    Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

    if (mpViewer) {
        mpViewer->RequestStop();
        while (!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    if (!bLocMap) {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestReset();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }

    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestReset();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clear();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearAtlas();
    mpAtlas->CreateNewMap();
    if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR ||
        mSensor == System::IMU_RGBD)
        mpAtlas->SetInertialSensor();
    mnInitialFrameId = 0;

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    mbReadyToInitializate = false;
    mbSetInit = false;

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    mCurrentFrame = Frame();
    mnLastRelocFrameId = 0;
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame *>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
    mvIniMatches.clear();

    if (mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::ResetActiveMap(bool bLocMap) {
    Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
    if (mpViewer) {
        mpViewer->RequestStop();
        while (!mpViewer->isStopped())
            usleep(3000);
    }

    Map *pMap = mpAtlas->GetCurrentMap();

    if (!bLocMap) {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_VERY_VERBOSE);
        mpLocalMapper->RequestResetActiveMap(pMap);
        Verbose::PrintMess("done", Verbose::VERBOSITY_VERY_VERBOSE);
    }

    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestResetActiveMap(pMap);
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearMap();

    // KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
    // Frame::nNextId = mnLastInitFrameId;
    mnLastInitFrameId = Frame::nNextId;
    // mnLastRelocFrameId = mnLastInitFrameId;
    mState = NO_IMAGES_YET; // NOT_INITIALIZED;

    mbReadyToInitializate = false;

    list<bool> lbLost;
    // lbLost.reserve(mlbLost.size());
    unsigned int index = mnFirstFrameId;
    cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
    for (Map *pMap : mpAtlas->GetAllMaps()) {
        if (pMap->GetAllKeyFrames().size() > 0) {
            if (index > pMap->GetLowerKFID())
                index = pMap->GetLowerKFID();
        }
    }

    // cout << "First Frame id: " << index << endl;
    int num_lost = 0;
    cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

    for (list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++) {
        if (index < mnInitialFrameId)
            lbLost.push_back(*ilbL);
        else {
            lbLost.push_back(true);
            num_lost += 1;
        }

        index++;
    }
    cout << num_lost << " Frames set to lost" << endl;

    mlbLost = lbLost;

    mnInitialFrameId = mCurrentFrame.mnId;
    mnLastRelocFrameId = mCurrentFrame.mnId;

    mCurrentFrame = Frame();
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame *>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
    mvIniMatches.clear();

    mbVelocity = false;

    if (mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

vector<MapPoint *> Tracking::GetLocalMapMPS() { return mvpLocalMapPoints; }

void Tracking::ChangeCalibration(const string &strSettingPath) {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    mK_.setIdentity();
    mK_(0, 0) = fx;
    mK_(1, 1) = fy;
    mK_(0, 2) = cx;
    mK_(1, 2) = cy;

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0) {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag) { mbOnlyTracking = flag; }

void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame *pCurrentKeyFrame) {
    Map *pMap = pCurrentKeyFrame->GetMap();
    unsigned int index = mnFirstFrameId;
    list<ORB_SLAM3::KeyFrame *>::iterator lRit = mlpReferences.begin();
    list<bool>::iterator lbL = mlbLost.begin();
    for (auto lit = mlRelativeFramePoses.begin(), lend = mlRelativeFramePoses.end(); lit != lend;
         lit++, lRit++, lbL++) {
        if (*lbL)
            continue;

        KeyFrame *pKF = *lRit;

        while (pKF->isBad()) {
            pKF = pKF->GetParent();
        }

        if (pKF->GetMap() == pMap) {
            (*lit).translation() *= s;
        }
    }

    mLastBias = b;

    mpLastKeyFrame = pCurrentKeyFrame;

    mLastFrame.SetNewBias(mLastBias);
    mCurrentFrame.SetNewBias(mLastBias);

    while (!mCurrentFrame.imuIsPreintegrated()) {
        usleep(500);
    }

    if (mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId) {
        mLastFrame.SetImuPoseVelocity(
            mLastFrame.mpLastKeyFrame->GetImuRotation(),
            mLastFrame.mpLastKeyFrame->GetImuPosition(),
            mLastFrame.mpLastKeyFrame->GetVelocity());
    } else {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const Eigen::Vector3f twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
        float t12 = mLastFrame.mpImuPreintegrated->dT;

        mLastFrame.SetImuPoseVelocity(
            IMU::NormalizeRotation(
                Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
            Vwb1 + Gz * t12 + Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    if (mCurrentFrame.mpImuPreintegrated) {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

        const Eigen::Vector3f twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
        float t12 = mCurrentFrame.mpImuPreintegrated->dT;

        mCurrentFrame.SetImuPoseVelocity(
            IMU::NormalizeRotation(
                Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
            Vwb1 + Gz * t12 + Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    mnFirstImuFrameId = mCurrentFrame.mnId;
}

void Tracking::NewDataset() { mnNumDataset++; }

int Tracking::GetNumberDataset() { return mnNumDataset; }

int Tracking::GetMatchesInliers() { return mnMatchesInliers; }

void Tracking::SaveSubTrajectory(
    string strNameFile_frames,
    string strNameFile_kf,
    string strFolder) {
    mpSystem->SaveTrajectoryEuRoC(strFolder + strNameFile_frames);
    // mpSystem->SaveKeyFrameTrajectoryEuRoC(strFolder + strNameFile_kf);
}

void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map *pMap) {
    mpSystem->SaveTrajectoryEuRoC(strNameFile_frames, pMap);
    if (!strNameFile_kf.empty())
        mpSystem->SaveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
}

float Tracking::GetImageScale() { return mImageScale; }

#ifdef REGISTER_LOOP
void Tracking::RequestStop() {
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
}

bool Tracking::Stop() {
    unique_lock<mutex> lock(mMutexStop);
    if (mbStopRequested && !mbNotStop) {
        mbStopped = true;
        cout << "Tracking STOP" << endl;
        return true;
    }

    return false;
}

bool Tracking::stopRequested() {
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

bool Tracking::isStopped() {
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

void Tracking::Release() {
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
    mbStopRequested = false;
}
#endif

} // namespace ORB_SLAM3
