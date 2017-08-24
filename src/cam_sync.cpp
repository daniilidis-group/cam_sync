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

#include <cam_sync/cam_sync.h>
#include <math.h>

namespace cam_sync {
  CamSync::CamSync(const ros::NodeHandle& parentNode)
    : parentNode_(parentNode) {
    configServer_.reset(new dynamic_reconfigure::Server<Config>(parentNode_));
    
    parentNode.getParam("num_cameras", numCameras_);

    double fps;
    parentNode.getParam("fps", fps);
    setFPS(fps);

    for (int i = 0; i < numCameras_; i++) {
      CamPtr cam_tmp = 
        boost::make_shared<Cam>(parentNode_, "cam" + std::to_string(i));
      cameras_.push_back(move(cam_tmp));
    }

    // for publishing the currently set exposure values
    configServer_->setCallback(boost::bind(&CamSync::configure, this, _1, _2));
  }
  CamSync::~CamSync()
  {
    stopPoll();
  }

  void CamSync::setFPS(double fps) {
    std::unique_lock<std::mutex> lock(timeMutex_);
    fps_ = fps;
    maxWait_ = std::chrono::nanoseconds((int64_t)(0.5 * 1e9 / fps_));
    //maxWait_ = std::chrono::nanoseconds((int64_t)(1000.0 * 1e9 / fps_));
  }


  void CamSync::start() {
    time_  = ros::Time::now();
    double duration(60);  // in seconds
    parentNode_.getParam("rec_length", duration);

    timer_ = parentNode_.createTimer(ros::Duration(duration),
                                     boost::bind(&CamSync::timerCallback,
                                                 this, _1), /*oneshot*/ true);
    startPoll();
  }

  void CamSync::timerCallback(const ros::TimerEvent &event) {
    ROS_INFO("timer expired, shutting down!");
    ros::shutdown();
    ROS_INFO("shutdown complete!");
  }

  
  void CamSync::configure(Config& config, int level) {
    ROS_INFO("configuring server!");
    if (level < 0) {
      ROS_INFO("%s: %s", parentNode_.getNamespace().c_str(),
               "Initializing reconfigure server");
    }
    setFPS(config.fps);

    CamConfig cc;
    cc.fps              = config.fps;
    //cc.video_mode       = 23; // format7
    cc.video_mode       = 0; // format7
    cc.format7_mode     = config.format7_mode;
    cc.width            = config.width;
    cc.height           = config.height;
    cc.raw_bayer_output = false;
    cc.trigger_source   = -1; // free running
    cc.pixel_format     = 22; // raw8
    cc.trigger_polarity = 0;
    cc.strobe_control   = -1;
    cc.strobe_polarity  = 0;
    cc.exposure         = false;
    cc.auto_exposure    = false;
    cc.exposure_value   = config.exposure_value;
    cc.auto_shutter     = false;
    cc.shutter_ms       = config.shutter_ms;
    cc.auto_gain        = false;
    cc.gain_db          = config.gain_db;
    cc.white_balance    = config.white_balance;
    cc.auto_white_balance = config.auto_white_balance;
    cc.wb_blue          = config.wb_blue;
    cc.wb_red           = config.wb_red;
    cc.brightness       = 0.0;
    cc.gamma            = 0.5;
    configureCams(cc);
  }

  void CamSync::configureCams(CamConfig& config) {
    if (stopPoll()) {
      // something was running
      configureCameras(config);
      startPoll();
    } else {
      // nothing was running
      configureCameras(config);
    }
  }

  void CamSync::frameGrabThread(int camIndex) {
    ROS_INFO("Starting up frame grab thread for camera %d", camIndex);

    // Grab a copy of the time, then can release the lock so that 
    // trigger can start a new frame if necessary.
    ros::Time lastTime;
    {
      std::unique_lock<std::mutex> lock(timeMutex_);
      lastTime = time_;
    }
    Time t0;
    while (ros::ok()) {
      auto image_msg = boost::make_shared<sensor_msgs::Image>();
      CamPtr curCam = cameras_[camIndex];
      t0 = ros::Time::now();
      bool ret = curCam->Grab(image_msg);
      bool timedOut(false);
      ros::Time t1 = ros::Time::now();
      {  // this section is protected by mutex
        
        std::unique_lock<std::mutex> lock(timeMutex_);
        if (camIndex == masterCamIdx_) {
          // master camera updates the timestamp for everybody
          time_ = ros::Time::now();
          timeCV_.notify_all();
        } else {
          // slave cameras wait until the master has published
          // a new timestamp.
          while (time_ <= lastTime) {
            // lock will be free while waiting!
            if (timeCV_.wait_for(lock, maxWait_) == std::cv_status::timeout) {
              // We timed out, what happened? In theory, we should only have to wait
              // a very short time until the master camera updates the global time stamp.
              // But wait, maybe there was on old frame in our buffer which we
              // retrieved when calling Grab(), so we didn't have to wait for the strobe,
              // grabbed the old frame and are now waiting for the master who is waiting for the
              // next frame. What to do? Let's see if there is another frame waiting
              // for us, and grab that one.
              timedOut = true;
              ret = curCam->GrabNonBlocking(image_msg);
            }
          }
          lastTime = time_;
        }
        image_msg->header.stamp = time_;
      }
      if (ret) {
        curCam->Publish(image_msg);
      } else {
        if (!timedOut) {
          ROS_ERROR("There was a problem grabbing a frame from cam%d", camIndex);
        } else {
          ROS_ERROR("Late frame from cam%d", camIndex);
        }
      }
      {
        std::unique_lock<std::mutex> lock(pollMutex_);
        if (!keepPolling_) break;
      }
    }
    ROS_INFO("frame grabbing thread exited!");
  }

  void CamSync::configureCameras(CamConfig& config) {
    if (masterCamIdx_ >= cameras_.size()) {
      ROS_ERROR("INVALID MASTER CAM INDEX: %d for %zu cams!",
                masterCamIdx_, cameras_.size());
      return;
    }
    // first set the master!
    config.fps             = fps_;
    config.strobe_control  = 2;  // GPIO 2
    config.strobe_polarity = 0;  // low
    config.trigger_source  = -1; // free running

#ifdef USE_AUTO_EXP
    config.exposure        = true;
    config.auto_shutter    = true;
    config.auto_gain       = true;
#endif

    CamConfig cc(config);
    cameras_[masterCamIdx_]->Stop();
    cameras_[masterCamIdx_]->camera().Configure(cc);
    cameras_[masterCamIdx_]->Start();
    

    // Switch on trigger for slave
    config.fps              = fps_ * 1.5;  // max frame rate!
    config.trigger_polarity = 0;   // low
    config.trigger_source   = 3;   // GPIO 3 (wired to GPIO 2 of master)
    config.trigger_mode     = 14;  // overlapped processing

    config.exposure         = false;
    config.auto_shutter     = false;
    config.auto_gain        = false;
    
    for (int i=0; i<numCameras_; ++i) {
      if (i == masterCamIdx_) continue;
      CamConfig cc2(config);
      CamPtr curCam = cameras_[i];
      curCam->Stop();
      curCam->camera().Configure(cc2);
      curCam->Start();
    }
  }

  bool CamSync::startPoll() {
    std::unique_lock<std::mutex> lock(pollMutex_);
    if (frameGrabThreads_.empty()) {
      keepPolling_ = true;
      for (int i = 0; i < numCameras_; ++i) {
        cameras_[i]->camera().StartCapture();
        frameGrabThreads_.push_back(
          boost::make_shared<boost::thread>(&CamSync::frameGrabThread, this, i));
      }
      return (true);
    }
    return (false);
  }

  bool CamSync::stopPoll() {
    {
      std::unique_lock<std::mutex> lock(pollMutex_);
      keepPolling_ = false;
    }
    if (!frameGrabThreads_.empty()) {
      // harvest the threads
      for (auto &th : frameGrabThreads_) {
        th->join();
      }
      frameGrabThreads_.clear();
      // must stop capture afterwards!
      for (auto &cam : cameras_) {
        cam->camera().StopCapture();
      }
      return (true);
    }
    return (false);
  }

}  // namespace poll_cameras
