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
#include <iomanip>


//#define SIMULATE_FRAME_DROPS

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

  static double get_pgr_timestamp(const flea3::Image &pgrImage) {
    //
    //    TimeStamp ts = image.GetTimeStamp();
    //    now look at ptgrey manuals for meaning of fields:
    //    ts.cycleSeconds, ts.cycleCount, ts.cycleOffset
    flea3::TimeStamp ts = pgrImage.GetTimeStamp();
    const double CYCLE_COUNT_TO_SEC = 0.000125; // 8kHz
    const double CYCLE_OFFSET_TO_SEC= CYCLE_COUNT_TO_SEC / 4096; // 12 bit
    double tstamp = ts.cycleSeconds + ts.cycleCount * CYCLE_COUNT_TO_SEC +
      ts.cycleOffset * CYCLE_OFFSET_TO_SEC;
    return (tstamp);
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

  CamSync::CameraFrame
  CamSync::FrameQueue::waitForNextFrame(bool *keepRunning) {
    CameraFrame frame;
    std::unique_lock<std::mutex> lock(mutex);
    const std::chrono::nanoseconds timeout((int64_t)(1000000000LL));
    while (ros::ok() && *keepRunning && frames.empty()) {
      cv.wait_for(lock, timeout);
    }
    if (!frames.empty()) {
      frame = frames.back();
      frames.pop_back();
    }
    return (frame);
  }

  void CamSync::FrameQueue::addFrame(const CameraFrame &frame) {
    std::unique_lock<std::mutex> lock(mutex);
    frames.push_front(frame);
    cv.notify_all();
  }

  void CamSync::FrameQueue::addFrame(int camId, bool ret, const ros::Time &ts,
                                     double camTs, const ImagePtr &msg) {
    std::unique_lock<std::mutex> lock(mutex);
    frames.push_front(CameraFrame(camId, ret, ts, ros::Time::now(),  camTs, msg));
    cv.notify_all();
  }

  CamSync::CamSync(const ros::NodeHandle& parentNode)
    : parentNode_(parentNode) {
    configServer_.reset(new dynamic_reconfigure::Server<Config>(parentNode_));
    
    parentNode.getParam("num_cameras", numCameras_);
    parentNode.param<int>("master_camera_index", masterCamIdx_, 0);
    parentNode.param<bool>("rotate_image", rotateImage_, false);
    parentNode.param<bool>("debug_timestamps", debugTimeStamps_, false);

    double fps;
    parentNode.getParam("fps", fps);
    setFPS(fps);

    for (int i = 0; i < numCameras_; i++) {
      std::string camName = "cam" + std::to_string(i);
      CamPtr cam_tmp = boost::make_shared<Cam>(parentNode_, i, camName);
      cameras_.push_back(move(cam_tmp));
      exposureControllers_.push_back(ControllerPtr(new ExposureController(parentNode_, camName)));
      timeGap_.push_back(0);
    }
    configServer_->setCallback(boost::bind(&CamSync::configure, this, _1, _2));
  }
  CamSync::~CamSync()
  {
    stopPoll();
  }

  void CamSync::setFPS(double fps) {
    fps_ = fps;
  }


  void CamSync::start() {
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

  void CamSync::framePublishThread(int camIndex) {
    CamPtr curCam = cameras_[camIndex];
    while (ros::ok() && keepPolling_) {
      CameraFrame frame = curCam->frames.waitForNextFrame(&keepPolling_);
      if (!frame.isValid) {
        continue;
      }
      if (rotateImage_) {
        curCam->Publish(rotate_image(frame.msg));
      } else {
        curCam->Publish(frame.msg);
      }
    }
    ROS_INFO_STREAM("frame publish thread done!");
  }
  
  bool CamSync::updateTimeStatistics(const CameraFrame &frame, ros::Time *tstamp) {
    if (lastTime_ == ros::Time(0)) {
      lastTime_ = frame.arrivalTime;
      return (false);
    }
    double dt = (frame.arrivalTime - lastTime_).toSec();
    lastTime_ = frame.arrivalTime;
    const auto &cam = cameras_[frame.cameraId];
    int camFramesAdvanced = cam->updateTimeStamp(frame.cameraTimeStamp);
    if (camFramesAdvanced != 1) {
      ROS_WARN_STREAM("camera " << frame.cameraId << " dropped frames: "
                      << (camFramesAdvanced - 1));
    }
    // position to new image counter
    imageCounter_ = (imageCounter_ + camFramesAdvanced) % cameras_.size();
    double const alpha = 0.1;
    timeGap_[imageCounter_] =  timeGap_[imageCounter_] * (1.0 - alpha) + dt * alpha;
    if (timeGap_[imageCounter_] > maxTimeGap_) {
      maxTimeGap_            = timeGap_[imageCounter_];
      maxTimeGapCounter_     = imageCounter_;
    }
    numCamFramesWithSameTimeStamp_ += camFramesAdvanced;
    if (imageCounter_ == maxTimeGapCounter_ || numCamFramesWithSameTimeStamp_ > cameras_.size()) {
      //  assume a new frame has started
      currentFrameTimeStamp_ = frame.arrivalTime;
      // update the maximum gap in case it actually shrinks
      maxTimeGap_ = timeGap_[imageCounter_];
      numCamFramesWithSameTimeStamp_ = 0;
    }
    *tstamp = currentFrameTimeStamp_;
#if 0    
    for (const auto &tg: timeGap_) {
      std::cout << " " << std::setw(6) << std::setprecision(4) << std::fixed << tg;
    }
    std::cout << std::endl;
#endif    
    return (true);
  }

  int CamSync::Cam::updateTimeStamp(double timeStamp) {
    if (lastCameraTime < 0 || fps < 0) {
      lastCameraTime = timeStamp;
      return (1);
    }
    if (lastCameraTime > timeStamp) lastCameraTime -= 128.0; // wrap around
    double nframes = std::round((timeStamp - lastCameraTime) * fps);
    lastCameraTime = timeStamp;
    return ((int)nframes);
  }

  void CamSync::timeStampThread() {
    std::ofstream tsFile;
    if (debugTimeStamps_) {
      tsFile.open("/tmp/ts.txt");
    }
    ros::Time t0 = ros::Time::now();
    const std::chrono::nanoseconds timeout((int64_t)(1000000000LL));
    while (ros::ok() && keepPolling_) {
      CameraFrame frame = cameraFrames_.waitForNextFrame(&keepPolling_);
      if (keepPolling_ && ros::ok()) {
        // got valid frame, send it to respective camera
        ros::Time timeStamp;
        if (updateTimeStatistics(frame, &timeStamp)) {
          frame.setMessageTimeStamp(timeStamp);
          if (debugTimeStamps_) {
            tsFile << (frame.arrivalTime - t0) << " "  << (timeStamp - t0) << " " << std::endl;
          }
          cameras_[frame.cameraId]->frames.addFrame(frame);
        }
      }
    }
  }

  void CamSync::frameGrabThread(int camIndex) {
    CamPtr curCam = cameras_[camIndex];
#ifdef SIMULATE_FRAME_DROPS    
    const int failureThreshold = (int)(RAND_MAX * 0.01);
#endif    
    while (ros::ok()) {
      auto image_msg = boost::make_shared<sensor_msgs::Image>();
      ros::Time t_grab_start = ros::Time::now();
      flea3::Image pgrImage;
      bool ret = curCam->camera().GrabImage(*image_msg, &pgrImage);
#ifdef SIMULATE_FRAME_DROPS      
      if (std::rand() < failureThreshold) {
        continue;
      }
#endif      
      double ts = get_pgr_timestamp(pgrImage);
      // put the new frame in the queue
      cameraFrames_.addFrame(camIndex, ret, t_grab_start, ts, image_msg);
      if (ret) {
        double newShutter(-1.0), newGain(-1.0);
        exposureControllers_[camIndex]->imageCallback(image_msg, &newShutter, &newGain);
        setShutter(curCam, newShutter);
        setGain(curCam, newGain);
      } else {
        ROS_WARN_STREAM("frame grab failed for camera " << camIndex);
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
      cameras_[masterCamIdx_]->setFPS(fps_);
      cameras_[masterCamIdx_]->camera().StartCapture();
      cameras_[masterCamIdx_]->camera().SetEnableTimeStamps(true);
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
        curCam->camera().SetEnableTimeStamps(true);
        exposureControllers_[i]->setFPS(config_.fps, config_.max_free_fps);
        exposureControllers_[i]->setCurrentShutter(cc2.shutter_ms);
        exposureControllers_[i]->setCurrentGain(cc2.gain_db);
        curCam->setFPS(fps_);
        curCam->camera().StartCapture();
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
