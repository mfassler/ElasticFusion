#include "RealSenseInterface.h"
#include <functional>

#include <stdio.h>

#ifdef WITH_REALSENSE
RealSenseInterface::RealSenseInterface(int inWidth,int inHeight,int inFps)
  : width(inWidth),
  height(inHeight),
  fps(inFps),
  initSuccessful(true)
{
  auto list = ctx.query_devices();
  if(list.size() == 0)
  {
    errorText = "No device connected.";
    initSuccessful = false;
    return;
  }

  dev = list.front();

  latestDepthIndex.assign(-1);
  latestRgbIndex.assign(-1);

  for(int i = 0; i < numBuffers; i++)
  {
    uint8_t * newImage = (uint8_t *)calloc(width * height * 3,sizeof(uint8_t));
    rgbBuffers[i] = std::pair<uint8_t *,int64_t>(newImage,0);
  }

  for(int i = 0; i < numBuffers; i++)
  {
    uint8_t * newDepth = (uint8_t *)calloc(width * height * 2,sizeof(uint8_t));
    uint8_t * newImage = (uint8_t *)calloc(width * height * 3,sizeof(uint8_t));
    frameBuffers[i] = std::pair<std::pair<uint8_t *,uint8_t *>,int64_t>(std::pair<uint8_t *,uint8_t *>(newDepth,newImage),0);
  }

  setAutoExposure(true);
  setAutoWhiteBalance(true);


  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8);
  cfg.enable_stream(RS2_STREAM_DEPTH, width, height);

  pipe.start(cfg);

  std::thread t([&]() {
    while (true)
    {
      printf("b4 wait_for_frames()\n");
      auto frames = pipe.wait_for_frames();
      printf("after wait_for_frames()\n");

      rs2::video_frame vid_frame = frames.get_color_frame();
      rs2::depth_frame dep_frame = frames.get_depth_frame();
      int bufferIndex;

      //  ************************************
      //   THIS WAS FROM RGBCallback()
      //  ************************************

      lastRgbTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();


      bufferIndex = (latestRgbIndex.getValue() + 1) % numBuffers;

      memcpy(rgbBuffers[bufferIndex].first, vid_frame.get_data(),
        vid_frame.get_width() * vid_frame.get_height() * 3);


      rgbBuffers[bufferIndex].second = lastRgbTime;

      latestRgbIndex++;


      //  ************************************
      //   THIS WAS FROM DepthCallback()
      //  ************************************
      lastDepthTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

      bufferIndex = (latestDepthIndex.getValue() + 1) % numBuffers;

      // The multiplication by 2 is here because the depth is actually uint16_t
      memcpy(frameBuffers[bufferIndex].first.first, dep_frame.get_data(),
        dep_frame.get_width() * dep_frame.get_height() * 2);


      frameBuffers[bufferIndex].second = lastDepthTime;

      int lastImageVal = latestRgbIndex.getValue();

      if(lastImageVal == -1)
      {
        return;
      }

      lastImageVal %= numBuffers;

      memcpy(frameBuffers[bufferIndex].first.second,rgbBuffers[lastImageVal].first,
        dep_frame.get_width() * dep_frame.get_height() * 3);

      latestDepthIndex++;

    }
  });
  t.detach();

}

RealSenseInterface::~RealSenseInterface()
{
  if(initSuccessful)
  {
    //dev->stop();
    pipe.stop();

    for(int i = 0; i < numBuffers; i++)
    {
      free(rgbBuffers[i].first);
    }

    for(int i = 0; i < numBuffers; i++)
    {
      free(frameBuffers[i].first.first);
      free(frameBuffers[i].first.second);
    }

  }
}

void RealSenseInterface::setAutoExposure(bool value)
{
  //dev->set_option(rs2::option::color_enable_auto_exposure, value);
  //auto color_sensor = dev->first<rs2::color_sensor>();
  auto color_sensor = dev.first<rs2::sensor>();
  if (color_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
    color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, value);
  }
}

void RealSenseInterface::setAutoWhiteBalance(bool value)
{
  //dev->set_option(rs2::option::color_enable_auto_white_balance, value);
  auto color_sensor = dev.first<rs2::sensor>();
  if (color_sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)) {
    color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, value);
  }
}

bool RealSenseInterface::getAutoExposure()
{
  //return dev->get_option(rs2::option::color_enable_auto_exposure);
  auto color_sensor = dev.first<rs2::sensor>();
  if (color_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
    return color_sensor.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE);
  }
  return false;
}

bool RealSenseInterface::getAutoWhiteBalance()
{
  //return dev->get_option(rs2::option::color_enable_auto_white_balance);
  auto color_sensor = dev.first<rs2::sensor>();
  if (color_sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)) {
    return color_sensor.get_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);
  }
  return false;
}
#else

RealSenseInterface::RealSenseInterface(int inWidth,int inHeight,int inFps)
  : width(inWidth),
  height(inHeight),
  fps(inFps),
  initSuccessful(false)
{
  errorText = "Compiled without Intel RealSense library";
}

RealSenseInterface::~RealSenseInterface()
{
}

void RealSenseInterface::setAutoExposure(bool value)
{
}

void RealSenseInterface::setAutoWhiteBalance(bool value)
{
}

bool RealSenseInterface::getAutoExposure()
{
  return false;
}

bool RealSenseInterface::getAutoWhiteBalance()
{
  return false;
}
#endif
