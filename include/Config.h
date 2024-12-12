#ifndef CONFIG_H
#define CONFIG_H

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

namespace ORB_SLAM3 {

class ViewerConfig {};

class CameraConfig {};

class ORBExtractorConfig {};

class IMUConfig {};

class ConfigParser {
  public:
    bool ParseConfigFile(std::string &strConfigFile);

  private:
    ViewerConfig mViewerConfig;
    CameraConfig mCameraConfig;
    ORBExtractorConfig mORBConfig;
    IMUConfig mIMUConfig;
};

} // namespace ORB_SLAM3

#endif // CONFIG_H
