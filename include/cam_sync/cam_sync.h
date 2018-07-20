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
  class Cam: public flea3::Flea3Ros {
  public:
    explicit Cam(const ros::NodeHandle& pnh,
                 int id,
                 const std::string& prefix = std::string());
    int id{0};
  };
  using CamPtr    = boost::shared_ptr<Cam>;
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

private:
  // Thread functions are private.
  void pollImages();
  void triggerThread();
  void timeStampThread();
  void frameGrabThread(int camIndex);
  void framePublishThread(int camIndex);
  void setFPS(double fps);
  void printStats();

  struct Stat {
    Stat(double dt = 0, unsigned long int c = 0ul) : sum(dt), cnt(c) {}
    void addDelay(double dt) { sum += dt; cnt++; }
    double getAverage() const { return (cnt > 0? sum/cnt :0); }
    void reset() { sum = 0; cnt = 0; }
    double sum;
    unsigned long int cnt;
  };

  struct CameraFrame {
    CameraFrame(bool iv = false, const ros::Time &ts = ros::Time(0), const ros::Time &te = ros::Time(0),
                const ImagePtr &img_msg = ImagePtr()) :
      isValid(iv), timeGrabStart(ts), timeGrabEnd(te), msg(img_msg) {
    }
    void update(bool ret, const ros::Time &ts, const ros::Time &te,
                const ImagePtr &img_msg);
    void release(const ros::Time &releaseTime, const ros::Time &tstamp);
    ImagePtr waitForNextFrame(bool *keepRunning);
    // --------------------------------------------
    bool        isValid{false};
    ros::Time   releaseTime;
    ros::Time   timeGrabStart;
    ros::Time   timeGrabEnd;
    ImagePtr    msg;
    std::mutex  mutex;
    std::condition_variable  cv;
    bool operator<(const CameraFrame& rhs) const {
      return (timeGrabEnd < rhs.timeGrabEnd);
    }
  };
  
  // Variables for the camera state
  ros::NodeHandle          parentNode_;
  int                      numCameras_;
  bool                     rotateImage_{false};
  int                      masterCamIdx_{0};
  Config                   config_;
  std::vector<Stat>        cameraStats_;
  std::deque<ros::Time>    arrivalTimes_;

  std::mutex               pollMutex_;
  bool                     keepPolling_{false};
  
  std::mutex               timeMutex_;
  std::condition_variable  timeCV_;
  ros::Time                t0_;
  ros::Time                time_;
  double                   fps_{15.0};
  ros::Duration            printInterval_{2.0};
  std::chrono::nanoseconds maxWait_;

  std::vector<CamPtr>      cameras_;
  std::deque<CameraFrame>  cameraFrames_;
  std::vector<ThreadPtr>   frameGrabThreads_;
  std::vector<ThreadPtr>   framePublishThreads_;
  ThreadPtr                timeStampThread_;
  typedef std::shared_ptr<ExposureController> ControllerPtr;
  std::vector<ControllerPtr> exposureControllers_;
  std::shared_ptr<dynamic_reconfigure::Server<Config> >    configServer_;
  ros::Timer               timer_;
  double                   currentMinFrameTime_{1e30};
  double                   avgMinFrameTime_{0};
  unsigned int             minFrameTimeCounter_{0};
  long                     index_;
};

}

#endif
