#pragma once

#include <string>
#include <iostream>
#include <algorithm>
#include <map>
#include <cstring>

#ifdef WITH_REALSENSE
#include "librealsense2/rs.hpp"
#endif

#include "ThreadMutexObject.h"
#include "CameraInterface.h"

class RealSenseInterface : public CameraInterface
{
public:
  RealSenseInterface(int width = 640,int height = 480,int fps = 30);
  virtual ~RealSenseInterface();

  int width,height,fps;

  bool getAutoExposure();
  bool getAutoWhiteBalance();
  virtual void setAutoExposure(bool value);
  virtual void setAutoWhiteBalance(bool value);

  virtual bool ok()
  {
    return initSuccessful;
  }

  virtual std::string error()
  {
    return errorText;
  }
  std::pair<uint8_t *,int64_t> rgbBuffers[numBuffers];

  // already declared in CameraInterface:
  //std::pair<std::pair<uint8_t *,uint8_t *>,int64_t> frameBuffers[numBuffers];

private:
#ifdef WITH_REALSENSE
  rs2::device dev;
  rs2::context ctx;
  rs2::pipeline pipe;
#endif

  bool initSuccessful;
  std::string errorText;
  
  ThreadMutexObject<int> latestRgbIndex;

  // already declared in CameraInterface:
  //ThreadMutexObject<int> latestDepthIndex;

  int64_t lastRgbTime;
  int64_t lastDepthTime;

};
