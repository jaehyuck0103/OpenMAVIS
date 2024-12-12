#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Atlas.h"
#include "MapPoint.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <mutex>
#include <unordered_set>

namespace ORB_SLAM3 {

class Tracking;
class Viewer;

class FrameDrawer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FrameDrawer(Atlas *pAtlas, const int sensor);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame(float imageScale = 1.f);
    cv::Mat DrawRightFrame(float imageScale = 1.f);
    cv::Mat DrawSideLeftFrame(float imageScale = 1.f);
    cv::Mat DrawSideRightFrame(float imageScale = 1.f);

    bool both;

    // Input sensor
    int mSensor;

  protected:
    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm, mImRight;
    cv::Mat mImSideLeft, mImSideRight;
    int N;
    vector<cv::KeyPoint> mvCurrentKeys, mvCurrentKeysRight;
    vector<cv::KeyPoint> mvCurrentKeysSideLeft, mvCurrentKeysSideRight;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;
    std::vector<float> mvCurrentDepth;
    float mThDepth;

    Atlas *mpAtlas;

    std::mutex mMutex;
    vector<pair<cv::Point2f, cv::Point2f>> mvTracks;

    Frame mCurrentFrame;
    vector<MapPoint *> mvpLocalMap;
    vector<cv::KeyPoint> mvMatchedKeys;
    vector<MapPoint *> mvpMatchedMPs;
    vector<cv::KeyPoint> mvOutlierKeys;
    vector<MapPoint *> mvpOutlierMPs;

    map<long unsigned int, cv::Point2f> mmProjectPoints;
    map<long unsigned int, cv::Point2f> mmMatchedInImage;
};

} // namespace ORB_SLAM3

#endif // FRAMEDRAWER_H
