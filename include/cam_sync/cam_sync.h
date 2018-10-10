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

#include <mutex>
#include <boost/thread.hpp>
#include <condition_variable>
#include <boost/shared_ptr.hpp>
#include <chrono>

#include <ros/ros.h>
#include <time.h>
#include <ros/message_traits.h>
#include <dynamic_reconfigure/server.h>
#include <flea3/Flea3DynConfig.h>
#include <flea3/flea3_ros.h>
#include <sensor_msgs/Image.h>
#include <deque>

namespace flea3 {
  class Flea3Ros;
}

namespace cam_sync {

class CamSync {
  
public:
  using ThreadPtr = boost::shared_ptr<boost::thread>;
  using ImagePtr  = boost::shared_ptr<sensor_msgs::Image>;
  using CamConfig = flea3::Flea3DynConfig;
  using Config    = CamSyncDynConfig;
  using Time      = ros::Time;

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
    CameraFrame(int camId = 0,
                bool iv = false, const ros::Time &ts = ros::Time(0),
                const ros::Time &ta = ros::Time(0),
                double camTS = 0,
                const FlyCapture2::ImageMetadata &md = FlyCapture2::ImageMetadata(),
                const ImagePtr &img_msg = ImagePtr()) :
      cameraId(camId), isValid(iv), timeGrabStart(ts), arrivalTime(ta),
      cameraTimeStamp(camTS), metaData(md), msg(img_msg) {
      arrivalTime = ros::Time::now();
    }
    void setMessageTimeStamp(const ros::Time &t) {
      if (msg) {
        msg->header.stamp = t;
      }
    }
    // --------------------------------------------
    int         cameraId;
    bool        isValid{false};
    bool        isPublished{false};
    ros::Time   releaseTime;
    ros::Time   timeGrabStart;
    ros::Time   arrivalTime;
    double      cameraTimeStamp;
    FlyCapture2::ImageMetadata metaData;
    ImagePtr    msg;
  };

  struct FrameQueue {
    FrameQueue(int ida=0) : id(ida) {};
    void         addFrame(const CameraFrame &f);
    void         addFrame(int camId, bool ret, const ros::Time &ts,
                          double camTs, const ImagePtr &msg,
                          const FlyCapture2::ImageMetadata &md);
    CameraFrame  waitForNextFrame(bool *keepRunning);
    int         id;
    std::mutex  mutex;
    std::condition_variable  cv;
    std::deque<CameraFrame> frames;
  };

  typedef std::shared_ptr<ExposureController> ControllerPtr;

  class Cam: public flea3::Flea3Ros {
  public:
    explicit Cam(const ros::NodeHandle& pnh,
                 int id,
                 const std::string& prefix = std::string());
    int  updateTimeStamp(double timeStamp);
    void setFPS(double f) { fps = f; }
    void publishMsg(const ImagePtr &imgMsg, const FlyCapture2::ImageMetadata &md);
    // ------------------
    int         id{0};
    FrameQueue  frames;
    double      lastCameraTime{-1.0};
    double      fps{-1.0};
    ControllerPtr exposureController;
    float       shutterRatio{1.0};
    float       gainRatio{1.0};
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
  bool updateTimeStatistics(const CameraFrame &frame, ros::Time *tstamp);

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
  // variables related to frame time keeping
  ros::Time                currentFrameTimeStamp_;
  std::vector<double>      timeGap_;
  double                   maxTimeGap_;
  int                      maxTimeGapCounter_{0};
  int                      imageCounter_{0};
  int                      numCamFramesWithSameTimeStamp_{0};
  ros::Time                lastTime_;
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
