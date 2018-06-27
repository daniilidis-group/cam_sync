/*
 * 2016 Bernd Pfrommer
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

namespace flea3 {
  class Flea3Ros;
}

namespace cam_sync {

class CamSync {
  
public:
  using Cam       = flea3::Flea3Ros;
  using CamPtr    = boost::shared_ptr<Cam>;
  using ThreadPtr = boost::shared_ptr<boost::thread>;
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
  void frameGrabThread(int camIndex);
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
  
  // Variables for the camera state
  ros::NodeHandle          parentNode_;
  int                      numCameras_;
  bool                     rotateImage_{false};
  int                      masterCamIdx_{0};
  Config                   config_;
  std::vector<Stat>        cameraStats_;

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
  std::vector<ThreadPtr>   frameGrabThreads_;
  typedef std::shared_ptr<ExposureController> ControllerPtr;
  std::vector<ControllerPtr> exposureControllers_;
  std::shared_ptr<dynamic_reconfigure::Server<Config> >    configServer_;
  ros::Timer               timer_;

  long index_;
};

}

#endif
