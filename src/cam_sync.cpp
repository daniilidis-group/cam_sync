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

#include "cam_sync/cam_sync.h"
#include "cam_sync/exposure_controller.h"
#include <math.h>

namespace cam_sync {
  static void setShutter(CamSync::CamPtr cam, double s) {
    bool auto_shutter(false);
    double rs(s);
    cam->camera().SetShutter(auto_shutter, rs);
    //ROS_INFO("set shutter to: %10.4fms, driver returned %8.4fms", s, rs);
  }

  static void setGain(CamSync::CamPtr cam, double g) {
    bool auto_gain(false);
    double rg(g);
    cam->camera().SetGain(auto_gain, rg);
    // ROS_INFO("set gain to:    %10.4fdb, driver returned %8.4fdb", g, rg);
  }

  static boost::shared_ptr<sensor_msgs::Image> 
  rotate_image(const boost::shared_ptr<sensor_msgs::Image> &msg) {
    boost::shared_ptr<sensor_msgs::Image> rotated(new sensor_msgs::Image());
    rotated->header = msg->header;
    rotated->width  = msg->height;
    rotated->height = msg->width;
    rotated->step   = rotated->width;
    rotated->encoding = msg->encoding;
    rotated->is_bigendian = msg->is_bigendian;
    rotated->data.resize(rotated->step * rotated->height);

    // need to skip a column to get the bayer pattern into the right shape!
    const int skip_cols = 1;
    const int line_size = msg->step;
    const int orig_width = msg->width;
    const int rotated_step = rotated->step;
    const int msg_height = msg->height;

    for (int row = 0; row < msg_height; row++) {
      const int row_offset = row * line_size + orig_width - 1;
      const int targ_offset = row - skip_cols * rotated_step;
      for (int col = skip_cols; col < orig_width - 1; col++) {
        rotated->data[col * rotated_step + targ_offset] = msg->data[row_offset - col];
      }
    }
    for (int row = 0; row < msg->height; row++) {
      const int col = 0;
      rotated->data[col * rotated->step + row] = msg->data[row * msg->step + (msg->width - col - 1)];
    }
    return (rotated);
  }

  CamSync::CamSync(const ros::NodeHandle& parentNode)
    : parentNode_(parentNode) {
    configServer_.reset(new dynamic_reconfigure::Server<Config>(parentNode_));
    
    parentNode.getParam("num_cameras", numCameras_);
    parentNode.param<int>("master_camera_index", masterCamIdx_, 0);
    parentNode.param<bool>("rotate_image", rotateImage_, false);
    double printInterval;
    parentNode.param<double>("print_interval", printInterval, 2.0);
    printInterval_ = ros::Duration(printInterval);

    double fps;
    parentNode.getParam("fps", fps);
    setFPS(fps);

    for (int i = 0; i < numCameras_; i++) {
      std::string camName = "cam" + std::to_string(i);
      CamPtr cam_tmp = boost::make_shared<Cam>(parentNode_, camName);
      cameras_.push_back(move(cam_tmp));
      cameraStats_.push_back(Stat(0.0, 0));
      exposureControllers_.push_back(ControllerPtr(new ExposureController(parentNode_, camName)));
    }
    configServer_->setCallback(boost::bind(&CamSync::configure, this, _1, _2));
  }
  CamSync::~CamSync()
  {
    stopPoll();
  }

  void CamSync::setFPS(double fps) {
    std::unique_lock<std::mutex> lock(timeMutex_);
    fps_ = fps;
    double T = 1.0 / fps_;
    double T_max_free = 1.0 / config_.max_free_fps + 0.005;
    double osTimeSlice = 0.020;
    double maxWaitSec = std::max(std::max(0.2 *T, T_max_free), osTimeSlice);
    maxWait_ = std::chrono::nanoseconds((int64_t)(1e9 * maxWaitSec));
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
    config_ = config;
    setFPS(config.fps);

    CamConfig cc;
    cc.fps              = config.fps;
    cc.video_mode       = 23; // format7
    //cc.video_mode       = 0; // format7
    cc.format7_mode     = config.format7_mode;
    cc.width            = config.width;
    cc.height           = config.height;
    cc.raw_bayer_output = false;
    cc.trigger_source   = config.trigger_source;
    cc.pixel_format     = 22; // raw8
    cc.trigger_polarity = config.trigger_polarity;
    cc.strobe_control   = config.strobe_control;
    cc.strobe_polarity  = config.strobe_polarity;
    cc.exposure         = false;
    cc.auto_exposure    = false;
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

  void CamSync::printStats() {
    double maxDelay(0), minDelay(1e10);
    int minIdx(-1), maxIdx(-1);
    
    for (unsigned int i = 0; i < cameraStats_.size(); i++) {
      if (i != masterCamIdx_)  {
        auto &stat = cameraStats_[i];
        double delay = stat.getAverage();
        if (delay > maxDelay) {
          maxDelay = delay;
          maxIdx = i;
        };
        if (delay < minDelay) {
          minDelay = delay;
          minIdx = i;
        }
        stat.reset();
      }
    }
    ROS_INFO("delay: min cam %2d %20.10fs, delay max cam %d %20.10fs", minIdx, minDelay,
             maxIdx, maxDelay);
  }

  void CamSync::frameGrabThread(int camIndex) {
    const bool isMaster(camIndex == masterCamIdx_ || (camIndex == 0 && masterCamIdx_ < 0));
    ROS_INFO("Starting up %s thread for camera %d, max wait is %.6f", isMaster? "MASTER" : "", camIndex, maxWait_.count()/1.0e6);
    CamPtr curCam = cameras_[camIndex];
    Stat   &stat  = cameraStats_[camIndex];
    // Grab a copy of the time, then can release the lock so that 
    // trigger can start a new frame if necessary.
    ros::Time lastTime, lastPrintTime;
    {
      std::unique_lock<std::mutex> lock(timeMutex_);
      lastTime = time_;
    }
    if (isMaster) {
      t0_ = ros::Time::now();
    }
    while (ros::ok()) {
      auto image_msg = boost::make_shared<sensor_msgs::Image>();
      ros::Time t1a = ros::Time::now();
      bool ret = curCam->Grab(image_msg);
      bool timedOut(false);
      ros::Time t1 = ros::Time::now();
      {  // this section is protected by mutex
        std::unique_lock<std::mutex> lock(timeMutex_);
        if (isMaster) {
          // master camera updates the timestamp for everybody
          time_ = ros::Time::now();
          timeCV_.notify_all();
          if (t1 > lastPrintTime + printInterval_) {
            printStats();
            lastPrintTime = t1;
          }
        } else {
          // slave cameras wait until the master has published
          // a new timestamp.
          while (time_ <= lastTime) {
            // lock will be free while waiting!
            if (timeCV_.wait_for(lock, maxWait_) == std::cv_status::timeout) {
              // if (false) {
              // We timed out, what happened? In theory, we should only have to wait
              // a very short time until the master camera updates the global time stamp.
              // But wait, maybe there was on old frame in our buffer which we
              // retrieved when calling Grab(), so we didn't have to wait for the strobe,
              // grabbed the old frame and are now waiting for the master who is waiting for the
              // next frame. What to do? Let's see if there is another frame waiting
              // for us, and grab that one.
              timedOut = true;
              ret = curCam->GrabNonBlocking(image_msg);
              ROS_WARN_STREAM("timeout cam " << camIndex << " max wait: " << maxWait_.count()/1e6  << "ms, regrabbed: " << ret);
            }
          }
          lastTime = time_;
        }
        image_msg->header.stamp = time_;
        //std::unique_lock<std::mutex> lock(timeMutex_);
        //std::cout << camIndex << " grab start: " << (t1a-t0_).toSec() << " grab return: " << (t1-t0_).toSec() << " got ts: " << (ros::Time::now()-t0_).toSec() << " msg tstamp: " << time_ << std::endl;
      }
      if (!isMaster) {
        ros::Time t2 = ros::Time::now();
        stat.addDelay((t2-t1).toSec());
      }
      if (ret) {
        if (rotateImage_) {
          curCam->Publish(rotate_image(image_msg));
        } else {
          curCam->Publish(image_msg);
        }
        double newShutter(-1.0), newGain(-1.0);
        exposureControllers_[camIndex]->imageCallback(image_msg, &newShutter, &newGain);
        if (newShutter != -1.0) {
          setShutter(curCam, newShutter);
        }
        if (newGain != -1.0) {
          setGain(curCam, newGain);
        }
      } else {
        if (!timedOut) {
          ROS_ERROR("driver could not grab a frame from cam%d", camIndex);
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
    if (masterCamIdx_ > 0 && masterCamIdx_ >= cameras_.size()) {
      ROS_ERROR("INVALID MASTER CAM INDEX: %d for %zu cams!",
                masterCamIdx_, cameras_.size());
      return;
    }
    //  save trigger source before overwriting
    int trigger_source = config.trigger_source;

    // first set the master!
    config.fps             = fps_;
    if (masterCamIdx_ > 0) {
      config.trigger_source  = -1; // free running
      //config.enable_output_voltage = 1; // for blackfly master
      config.exposure        = false;
      config.auto_shutter    = false;
      config.auto_gain       = false;

      CamConfig cc(config);
      cameras_[masterCamIdx_]->Stop();
      cameras_[masterCamIdx_]->camera().Configure(cc);
      exposureControllers_[masterCamIdx_]->setFPS(config_.fps, config_.max_free_fps);
      exposureControllers_[masterCamIdx_]->setCurrentShutter(cc.shutter_ms);
      exposureControllers_[masterCamIdx_]->setCurrentGain(cc.gain_db);
      cameras_[masterCamIdx_]->Start();
    }
    
    // Switch on trigger for slave
    config.fps              = fps_ * 1.5;  // max frame rate!
    config.trigger_source   = trigger_source; // restore
    config.enable_output_voltage = 0; // not needed for slaves

    config.strobe_control   = -1;  // no strobe control for slave
    config.trigger_mode     = 14;  // overlapped processing

    for (int i = 0; i < numCameras_; ++i) {
      if (i != masterCamIdx_) {
        CamConfig cc2(config);
        CamPtr curCam = cameras_[i];
        curCam->Stop();
        curCam->camera().Configure(cc2);
        exposureControllers_[i]->setFPS(config_.fps, config_.max_free_fps);
        exposureControllers_[i]->setCurrentShutter(cc2.shutter_ms);
        exposureControllers_[i]->setCurrentGain(cc2.gain_db);
        curCam->Start();
      }
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
