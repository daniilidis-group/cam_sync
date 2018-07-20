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

#include "cam_sync/cam_sync.h"
#include "cam_sync/exposure_controller.h"
#include <math.h>
#include <fstream>
#include <boost/range/irange.hpp>

namespace cam_sync {
  using boost::irange;

  static void setShutter(CamSync::CamPtr cam, double s) {
    if (s == -1.0) return;
    bool auto_shutter(false);
    double rs(s);
    cam->camera().SetShutter(auto_shutter, rs);
    //ROS_INFO("set shutter to: %10.4fms, driver returned %8.4fms", s, rs);
  }

  static void setGain(CamSync::CamPtr cam, double g) {
    if (g == -1.0) return;
    bool auto_gain(false);
    double rg(g);
    cam->camera().SetGain(auto_gain, rg);
    // ROS_INFO("set gain to:    %10.4fdb, driver returned %8.4fdb", g, rg);
  }

  static CamSync::ImagePtr
  rotate_image(const CamSync::ImagePtr &msg) {
    CamSync::ImagePtr rotated(new sensor_msgs::Image());
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

  CamSync::Cam::Cam(const ros::NodeHandle& pnh,
                    int i,
                    const std::string& prefix) : Flea3Ros(pnh, prefix), id(i) {
  }

  void CamSync::CameraFrame::update(bool ret,
                                   const ros::Time &ts, const ros::Time &te,
                                   const ImagePtr &img_msg) {
    std::unique_lock<std::mutex> lock(mutex);
    timeGrabStart = ts;
    timeGrabEnd   = te;
    msg           = img_msg;
    isValid       = ret;
  }

  void
  CamSync::CameraFrame::release(const ros::Time &relT,
                                const ros::Time &tstamp) {
    std::unique_lock<std::mutex> lock(mutex);
    releaseTime = relT;
    if (isValid) {
      msg->header.stamp = tstamp;
      // signal publishing thread to pick up
    }
    cv.notify_all();
  }

  CamSync::ImagePtr
  CamSync::CameraFrame::waitForNextFrame(bool *keepRunning) {
    std::unique_lock<std::mutex> lock(mutex);
    const std::chrono::nanoseconds timeout((int64_t)(1000000000LL));
    while (ros::ok() && *keepRunning && !isValid) {
      cv.wait_for(lock, timeout);
    }
    if (isValid) {
      isValid = false; // mark as consumed
      return (msg);
    }
    return (ImagePtr(NULL));
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
      CamPtr cam_tmp = boost::make_shared<Cam>(parentNode_, i, camName);
      cameras_.push_back(move(cam_tmp));
      cameraStats_.push_back(Stat(0.0, 0));
      exposureControllers_.push_back(ControllerPtr(new ExposureController(parentNode_, camName)));
    }
    cameraFrames_.resize(cameras_.size());
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
    // make a guess on the average minimum frame time
    avgMinFrameTime_ = 0.9 * T;
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

  void CamSync::framePublishThread(int camIndex) {
    ROS_INFO_STREAM("starting publish thread for " << camIndex);
    CamPtr curCam = cameras_[camIndex];
    while (ros::ok() && keepPolling_) {
      ImagePtr msg = cameraFrames_[camIndex].waitForNextFrame(&keepPolling_);
      if (!msg) {
        std::cout << "cam: " << camIndex << " got null frame " << std::endl;
        continue;
      }
      if (rotateImage_) {
        curCam->Publish(rotate_image(msg));
      } else {
        curCam->Publish(msg);
      }
    }
    ROS_INFO_STREAM("frame publish thread done!");
  }

  void CamSync::timeStampThread() {
    ROS_INFO("Starting up time sync thread");
    double maxWaitSec = 1.0;
    std::ofstream tsFile("/tmp/ts.txt");
    ros::Time t0 = ros::Time::now();
    const std::chrono::nanoseconds timeout((int64_t)(1000000000LL));
    while (ros::ok()) {
      // this section is protected by mutex
      std::unique_lock<std::mutex> lock(timeMutex_);
      if (timeCV_.wait_for(lock, timeout) == std::cv_status::timeout) {
        // we got no frame, check if we should still be running
        std::unique_lock<std::mutex> lock(pollMutex_);
        if (!keepPolling_) {
          return; // we are done here!
        }
      } else {
        if (arrivalTimes_.size() >= cameras_.size()) {
          // a frame from each camera has arrived (or multiple from the same
          // one in case of camera frame drops, but that should be rare)
          double frameTime = (arrivalTimes_.front() - arrivalTimes_.back()).toSec();
          arrivalTimes_.pop_back(); // remove oldest time
          tsFile << arrivalTimes_.front() - t0 << " " << frameTime << std::endl;
          if (frameTime < currentMinFrameTime_) {
            std::cout << arrivalTimes_.size() << " frame time: " << frameTime << " is less than min: " << currentMinFrameTime_ << std::endl;
            currentMinFrameTime_ = frameTime;
          }
          // every so many full frames, compound the minimum time into
          // the running average
          if (++minFrameTimeCounter_ >= cameras_.size() * 10) {
            const double alpha = 0.1;
            avgMinFrameTime_ = avgMinFrameTime_ * (1.0-alpha) + currentMinFrameTime_ * alpha;
            std::cout << "min frame time: " << currentMinFrameTime_ << " avg: " <<
              avgMinFrameTime_ << " T: " << 1.0 / fps_ << std::endl;

            minFrameTimeCounter_ = 0;
            currentMinFrameTime_ = 1e30; // reset minimum
          }
        }
        // if all cameras have frames, publish!
        int num_valid(0);
        for (const auto &f: cameraFrames_) {
          if (f.isValid) {
            num_valid++;
          }
        }
        if (num_valid == cameraFrames_.size()) {
          ros::Time t = ros::Time::now();
          ros::Time tstamp = t;
          for (const auto cam_idx: irange(0ul, cameraFrames_.size())) {
            auto &f = cameraFrames_[cam_idx];
            f.release(t, tstamp);
            //tsFile << (f.timeGrabEnd - t0) << " " << cam_idx << " " << (f.timeGrabStart - t0) << " " << (t - t0) << std::endl;
          }
        }
      }
    }
  }

  void CamSync::frameGrabThread(int camIndex) {
    ROS_INFO("Starting upthread for camera %d", camIndex);
    CamPtr curCam = cameras_[camIndex];
    while (ros::ok()) {
      auto image_msg = boost::make_shared<sensor_msgs::Image>();
      ros::Time t_grab_start = ros::Time::now();
      bool ret = curCam->Grab(image_msg);
      // put the new frame in the table
      {
        // this section is protected by mutex
        std::unique_lock<std::mutex> lock(timeMutex_);
        ros::Time t_grab_end = ros::Time::now();
        arrivalTimes_.push_front(t_grab_end);
        // XXX cameraFrames_[camIndex].update(ret, t_grab_start, t_grab_end, image_msg);

        // notify the timing thread of arrival
        timeCV_.notify_all();
      }
      if (ret) {
        double newShutter(-1.0), newGain(-1.0);
        exposureControllers_[camIndex]->imageCallback(image_msg, &newShutter, &newGain);
        setShutter(curCam, newShutter);
        setGain(curCam, newGain);
      } else {
        ROS_WARN_STREAM("grab thread failed for camera " << camIndex);
      }
      {
        std::unique_lock<std::mutex> lock(pollMutex_);
        if (!keepPolling_) break;
      }
    }
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
      cameras_[masterCamIdx_]->Start(NULL, NULL);
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
        curCam->Start(NULL, NULL);
      }
    }
  }

  bool CamSync::startPoll() {
    std::unique_lock<std::mutex> lock(pollMutex_);
    if (frameGrabThreads_.empty()) {
      keepPolling_ = true;
      for (int i = 0; i < numCameras_; ++i) {
        cameras_[i]->camera().StartCapture(NULL, NULL);
        frameGrabThreads_.push_back(
          boost::make_shared<boost::thread>(&CamSync::frameGrabThread, this, i));
        framePublishThreads_.push_back(
          boost::make_shared<boost::thread>(&CamSync::framePublishThread, this, i));
      }
      if (!timeStampThread_) {
        timeStampThread_ =
          boost::make_shared<boost::thread>(&CamSync::timeStampThread, this);
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
      for (auto &th : framePublishThreads_) {
        th->join();
      }
      frameGrabThreads_.clear();
      framePublishThreads_.clear();
      if (timeStampThread_) {
        timeStampThread_->join();
        timeStampThread_.reset();
      }
      // must stop capture afterwards!
      for (auto &cam : cameras_) {
        cam->camera().StopCapture();
      }
      return (true);
    }
    return (false);
  }

}  // namespace poll_cameras
