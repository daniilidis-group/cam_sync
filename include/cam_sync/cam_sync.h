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
  using ThreadPtr    = boost::shared_ptr<boost::thread>;
  using ImagePtr     = sensor_msgs::ImagePtr;
  using CamConfig    = flea3::Flea3DynConfig;
  using Config       = CamSyncDynConfig;
  using WallTime     = ros::WallTime;
  using WallDuration = ros::WallDuration;
  //
  // GlobalTime keeps track of a unified global time
  // that is used establish the time stamp for the
  // outgoing frames
  
  class GlobalTime {
  public:
    GlobalTime() : sumOffsets_{0}, numOffsets_{0} {};
    WallTime findFrameTime(const WallTime &cameraTime,
                           const double dtAvg);

    double getAvg() const  {
      return (numOffsets_ > 0 ? sumOffsets_/numOffsets_ : 0);
    }
    void resetOffset() {
      numOffsets_ = 0;
      sumOffsets_ = 0;
    }
    void recordOffset(double tdiff) {
      sumOffsets_ += tdiff;
      numOffsets_++;
    }
  private:
    std::list<WallTime> frameTimes_;
    double sumOffsets_{0};
    int    numOffsets_{0};
  };

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
    CameraFrame(int camId = 0, bool iv = false,
                const WallTime &tgrab = WallTime(0),
                const WallTime &ta    = WallTime(0),
                double camClock = 0,
                const FlyCapture2::ImageMetadata &md =
                FlyCapture2::ImageMetadata(),
                const ImagePtr &img_msg = ImagePtr()) :
      cameraId(camId), isValid(iv),
      grabStartTime(tgrab), arrivalTime(ta),
      imageTime(ta), cameraClock(camClock),
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
    WallTime      grabStartTime;
    WallTime      arrivalTime;
    WallTime      imageTime;
    double        cameraClock;
    FlyCapture2::ImageMetadata metaData;
    ImagePtr      msg;
  };

  typedef std::shared_ptr<CameraFrame> CameraFramePtr;

  struct FrameQueue {
    FrameQueue(int ida=0) : id(ida) {};
    void            addFrame(const CameraFramePtr &f);
    CameraFramePtr  waitForNextFrame(bool *keepRunning);
    // 
    int         id;
    std::mutex  mutex;
    std::condition_variable  cv;
    std::deque<CameraFramePtr> frames;
    bool        hasQueueBuildup{false};
  };

  typedef std::shared_ptr<ExposureController> ControllerPtr;
  

  class Cam: public flea3::Flea3Ros {
  public:
    explicit Cam(const ros::NodeHandle& pnh, int id,
                 int lagThresh,  double arrivalToCamTimeCoeff, bool debug,
                 const std::string& prefix = std::string());
    FrameQueue    &getFrames() { return (frames_); }
    inline bool timeStatsInitialized() const {
      return (lastArrivalTime_ != WallTime(0) && fps_ > 0);
    }
    void    setCameraTime(const WallTime &t) { cameraTime_ = t; }
    void    initializeTimeStats(const CameraFrame &f);
    void    updateImageTime(const CameraFramePtr &fp);
    double  gotNewFrame(const CameraFrame &f, double dtA, int *nframes);
    bool    updateCameraTime(const CameraFrame &f, int nframes, double dtAvg,
                             WallTime *frameTime,  GlobalTime *globalTime);
    void          setFPS(double f);
    void          publishMsg(const ImagePtr &imgMsg,
                             const FlyCapture2::ImageMetadata &md);
    ControllerPtr getExposureController() { return (exposureController_); }
    void          logStats(double dt);
    bool          isWarmedUp(const WallTime &t) const;

  private:
    // ------------------ variables
    int           id_{0};
    FrameQueue    frames_;
    ControllerPtr exposureController_;
    double        fps_{-1.0};
    float         shutterRatio_{1.0};
    float         gainRatio_{1.0};
    unsigned int  lastFrameCount_{0};
    WallTime      lastArrivalTime_{WallTime(0)};
    WallTime      cameraTime_{WallTime(0)};
    WallTime      firstArrivalTime_{WallTime(0)};
    WallTime      lastFrameTime_{0};
    WallTime      imageTime_{WallTime(0)};
    double        lastCameraClock_{0};
    double        offset_{0};
    unsigned int  frameCount_{0};
    unsigned int  framesDropped_{0};
    int           lagCounter_{0};
    int           lagThreshold_{10};
    double        variance_{0};
    bool          debug_{true};
    double        arrivalToCameraTimeCoeff_{0.005};
    std::ofstream debugFile_;
  };

  using CamPtr = boost::shared_ptr<Cam>;

private:
  void timeStampThread();
  void frameGrabThread(int camIndex);
  void framePublishThread(int camIndex);
  void setFPS(double fps);
  bool updateTimeStatistics(const CameraFramePtr &frame, WallTime *frameTime);

  // Variables for the camera state
  ros::NodeHandle          nh_;
  int                      numCameras_;
  bool                     rotateImage_{false};
  int                      masterCamIdx_{0};
  Config                   config_;

  std::mutex               pollMutex_;
  bool                     keepPolling_{false};
  
  double                   fps_{15.0};
  double                   dtAvgConst_;
  double                   dt_{0.025};
  double                   dtVar_{3e-5};
  double                   dtVarThreshold_{0};
  bool                     warmedUp_{false};

  std::vector<CamPtr>      cameras_;
  FrameQueue               cameraFrames_; // frames before dispatch
  GlobalTime               globalTime_;   // keeps track of global time

  WallTime                 lastLogTime_;
  WallDuration             logInterval_;
  
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
