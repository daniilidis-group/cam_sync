/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CAMSYNC_CAMSYNC_H
#define CAMSYNC_CAMSYNC_H

#include "cam_sync/exposure_controller.h"
#include "cam_sync/CamSyncDynConfig.h"

#include <flea3/Flea3DynConfig.h>
#include <flea3/flea3_ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <ros/message_traits.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <chrono>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <fstream>

#include <time.h>

namespace flea3 {
  class Flea3Ros;
}

namespace cam_sync {

class CamSync {
  
public:
  using ThreadPtr = boost::shared_ptr<boost::thread>;
  using ImagePtr  = sensor_msgs::ImagePtr;
  using CamConfig = flea3::Flea3DynConfig;
  using Config    = CamSyncDynConfig;
  using WallTime  = ros::WallTime;

  CamSync(const ros::NodeHandle& parentNode);
  ~CamSync();
  CamSync(const CamSync&) = delete;
  CamSync& operator=(const CamSync&) = delete;

  void configureCams(CamConfig& config);
  void configure(Config& config, int level);

  void start();
  bool startPoll();
  bool stopPoll();
  void configureCameras(CamConfig& config);
  void timerCallback(const ros::TimerEvent &event);

  struct CameraFrame {
    CameraFrame(int camId = 0, bool iv = false, const WallTime &tgrab = WallTime(0),
                const WallTime &ta = WallTime(0),
                unsigned int frameCnt = 0,
                const FlyCapture2::ImageMetadata &md =
                FlyCapture2::ImageMetadata(),
                const ImagePtr &img_msg = ImagePtr()) :
      cameraId(camId), isValid(iv), timeGrabStart(tgrab), arrivalTime(ta),
      cameraFrameCount(frameCnt),
      metaData(md), msg(img_msg) {
      arrivalTime = ta;
    }
    void setMessageTimeStamp(const WallTime &t) {
      if (msg) {
        msg->header.stamp = ros::Time(t.sec, t.nsec);
      }
    }
    // --------------------------------------------
    int           cameraId;
    bool          isValid{false};
    bool          isPublished{false};
    WallTime      releaseTime;
    WallTime      timeGrabStart;
    WallTime      arrivalTime;
    unsigned int  cameraFrameCount;
    FlyCapture2::ImageMetadata metaData;
    ImagePtr      msg;
  };

  struct FrameQueue {
    FrameQueue(int ida=0) : id(ida) {};
    void         addFrame(const CameraFrame &f);
    void         addFrame(int camId, bool ret, const WallTime &ts,
                          const WallTime &arrivalTime,
                          unsigned int frameCnt,
                          const ImagePtr &msg,
                          const FlyCapture2::ImageMetadata &md);
    CameraFrame  waitForNextFrame(bool *keepRunning);
    int         id;
    std::mutex  mutex;
    std::condition_variable  cv;
    std::deque<CameraFrame> frames;
    bool        hasQueueBuildup{false};
  };

  typedef std::shared_ptr<ExposureController> ControllerPtr;

  class Cam: public flea3::Flea3Ros {
  public:
    explicit Cam(const ros::NodeHandle& pnh, int id, bool debug,
                 const std::string& prefix = std::string());
    bool     updateCameraTime(unsigned int frameCount,
                              const WallTime &arrivalTime,
                              WallTime *frameTime);
    void     setFPS(double f);
    void     publishMsg(const ImagePtr &imgMsg,
                        const FlyCapture2::ImageMetadata &md);
    FrameQueue &getFrames() { return (frames_); }
    ControllerPtr getExposureController() { return (exposureController_); }
    void     logStats(double dt);
  private:
    // ------------------
    int           id_{0};
    FrameQueue    frames_;
    ControllerPtr exposureController_;
    double        fps_{-1.0};
    float         shutterRatio_{1.0};
    float         gainRatio_{1.0};
    unsigned int  lastFrameCount_{0};
    WallTime      lastArrivalTime_ {WallTime(0)};
    WallTime      cameraTime_ {WallTime(0)};
    double        dt_{0.03};
    double        offset_{0};
    unsigned int  frameCount_{0};
    unsigned int  framesDropped_{0};
    double        variance_{0};
    bool          debug_{true};
    std::ofstream debugFile_;
  };

  using CamPtr    = boost::shared_ptr<Cam>;

private:
  // Thread functions are private.
  void pollImages();
  void triggerThread();
  void timeStampThread();
  void frameGrabThread(int camIndex);
  void framePublishThread(int camIndex);
  void setFPS(double fps);
  bool updateTimeStatistics(const CameraFrame &frame);

  // Variables for the camera state
  ros::NodeHandle          parentNode_;
  int                      numCameras_;
  bool                     rotateImage_{false};
  int                      masterCamIdx_{0};
  Config                   config_;

  std::mutex               pollMutex_;
  bool                     keepPolling_{false};
  
  double                   fps_{15.0};

  std::vector<CamPtr>      cameras_;
  // frames detected
  FrameQueue               cameraFrames_;
  WallTime                 currentFrameTime_{WallTime(0)};
  WallTime                 lastLogTime_;
  ros::WallDuration        logInterval_;
  
  bool                     debugTimeStamps_{false};
  
  std::shared_ptr<dynamic_reconfigure::Server<Config>> configServer_;
  // timer for shutdown
  ros::Timer               timer_;
  // threads for frame grabbing, time keeping, and publishing
  std::vector<ThreadPtr>   frameGrabThreads_;
  std::vector<ThreadPtr>   framePublishThreads_;
  ThreadPtr                timeStampThread_;
};
  
}

#endif
