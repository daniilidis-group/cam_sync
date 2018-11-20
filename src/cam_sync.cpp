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
#include <math.h>
#include <fstream>
#include <boost/range/irange.hpp>
#include <iomanip>
#include <algorithm>


//#define SIMULATE_FRAME_DROPS

namespace cam_sync {
  using boost::irange;

  static ros::WallTime STARTUP_TIME = ros::WallTime::now();

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
        rotated->data[col * rotated_step + targ_offset] =
          msg->data[row_offset - col];
      }
    }
    for (int row = 0; row < msg->height; row++) {
      const int col = 0;
      rotated->data[col * rotated->step + row] =
        msg->data[row * msg->step + (msg->width - col - 1)];
    }
    return (rotated);
  }

  CamSync::Cam::Cam(const ros::NodeHandle& pnh,
                    int i, bool debug, const std::string& prefix) :
    Flea3Ros(pnh, prefix), id_(i), frames_(i), debug_(debug) {
    exposureController_.reset(new ExposureController(pnh, prefix));
    shutterRatio_    = camera().GetAbsToRelativeRatio(0x918, 0x81c);
    gainRatio_       = camera().GetAbsToRelativeRatio(0x928, 0x820);
    if (debug_) {
      debugFile_.open("cam_" + std::to_string(i) + ".txt");
    }
  }

  void CamSync::Cam::publishMsg(const ImagePtr &imgMsg,
                                const FlyCapture2::ImageMetadata &md) {
    Publish(imgMsg); // camera base method
    MetaData msg;
    msg.header        = imgMsg->header;
    msg.timestamp     = md.embeddedTimeStamp;
    msg.frame_counter = md.embeddedFrameCounter;
    msg.shutter       = (float)(md.embeddedShutter & 0x00000FFF) * shutterRatio_;
    msg.gain          = (float)(md.embeddedGain    & 0x00000FFF) * gainRatio_;
    msg.wb_red        = md.embeddedWhiteBalance & 0x00000FFF;
    msg.wb_blue       = (md.embeddedWhiteBalance & 0x00FFF000) >> 12;
    exposureController_->publish(msg);
  }

  void CamSync::Cam::logStats(double dt) {
    // NOTE: not taking lock during access, may be corrupted.
    //       It's just statistics after all
    const double dtInv    = 1.0 / dt;
    const double fps      = frameCount_    * dtInv;
    const double dropRate = framesDropped_ * 100.0 / frameCount_;
    const double offset   = offset_  * 100.0;
    const double jitter   = std::sqrt(variance_ / frameCount_) * 100.0;
    ROS_INFO("cam %02d fps: %7.3f drop: %8.4f %%  offset: %7.2f %%"
             " jitter: %6.2f %%", id_, fps, dropRate, offset, jitter);
    framesDropped_        = 0;
    frameCount_           = 0;
    variance_             = 0;
  }

  CamSync::CameraFrame
  CamSync::FrameQueue::waitForNextFrame(bool *keepRunning) {
    CameraFrame frame;
    std::unique_lock<std::mutex> lock(mutex);
    const std::chrono::microseconds timeout((int64_t)(10000000LL));
    while (ros::ok() && *keepRunning && frames.empty()) {
      cv.wait_for(lock, timeout);
    }
    if (!frames.empty()) {
      frame = frames.back();
      frames.pop_back();
      if (frames.size() > 50) {
        ROS_WARN_STREAM_THROTTLE(10, "cam " << id << " has queue of size "
                                 << frames.size());
        hasQueueBuildup = true;
      }
      if (hasQueueBuildup && frames.empty()) {
        ROS_WARN_STREAM_THROTTLE(10, "cam " << id << " has clear queue again");
        hasQueueBuildup = false;
      }
    }
    return (frame);
  }

  void CamSync::FrameQueue::addFrame(const CameraFrame &frame) {
    std::unique_lock<std::mutex> lock(mutex);
    frames.push_front(frame);
    cv.notify_all();
  }

  void CamSync::FrameQueue::addFrame(int camId, bool ret, const WallTime &tgrab,
      const WallTime &arrivalTime, unsigned int frameCnt,
      const ImagePtr &msg, const FlyCapture2::ImageMetadata &md) {
    std::unique_lock<std::mutex> lock(mutex);
    frames.push_front(CameraFrame(camId, ret, tgrab, arrivalTime, frameCnt,
                                  md, msg));
    cv.notify_all();
  }

  CamSync::CamSync(const ros::NodeHandle& parentNode)
    : parentNode_(parentNode) {
    configServer_.reset(new dynamic_reconfigure::Server<Config>(parentNode_));
    
    parentNode.getParam("num_cameras", numCameras_);
    parentNode.param<int>("master_camera_index", masterCamIdx_, 0);
    parentNode.param<bool>("rotate_image", rotateImage_, false);
    parentNode.param<bool>("debug_timestamps", debugTimeStamps_, false);
    double logInterval;
    parentNode.param<double>("logging_interval", logInterval, 60.0);
    logInterval_ = ros::WallDuration(logInterval);
    double fps(30.0);
    parentNode.getParam("fps", fps);
    setFPS(fps);
    double dtAvgTime;
    parentNode.param<double>("dt_avg_time", dtAvgTime, 10.0);
    dtAvgConst_ = 1.0 / (dtAvgTime * fps * numCameras_);
    dt_         = 1.0 / fps;  // starting estimate for average

    for (int i = 0; i < numCameras_; i++) {
      std::string camName = "cam" + std::to_string(i);
      CamPtr camTmp = boost::make_shared<Cam>(parentNode_, i,
                                              debugTimeStamps_, camName);
      cameras_.push_back(move(camTmp));
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
      CameraFrame frame = curCam->getFrames().waitForNextFrame(&keepPolling_);
      if (!frame.isValid) {
        continue;
      }
      if (rotateImage_) {
        curCam->publishMsg(rotate_image(frame.msg), frame.metaData);
      } else {
        curCam->publishMsg(frame.msg, frame.metaData);
      }
    }
    ROS_INFO_STREAM("frame publish thread done!");
  }

  bool CamSync::updateTimeStatistics(const CameraFrame &frame) {
    const auto &cam = cameras_[frame.cameraId];
    const double dt = cam->updateCameraTime(frame.cameraFrameCount,
                                            frame.arrivalTime,
                                            dt_, &currentFrameTime_);
    const double lowLim(0.3 * dt_), upLim(1.7 * dt_);
    const double dtc = (dt < lowLim) ? lowLim : ((dt > upLim) ? upLim : dt);
    dt_ = dt_ * (1.0 - dtAvgConst_) + dtAvgConst_ * dt;
    return (true);
  }

  void CamSync::Cam::setFPS(double f) {
    fps_ = f;
  }

  /*
   * updateCameraTime() primarily maintains a per-camera clock.
   * This is done by using the frame numbers that are embedded
   * in the image, and the wall clock arrival times of the images.
   * The global frame time is updated here as well, by whichever camera
   * gets the new frame first.
   */
  double
  CamSync::Cam::updateCameraTime(unsigned int frameCount,
                                 const WallTime &arrivalTime,
                                 double dtAvg, 
                                 WallTime *currentFrameTime) {
    if (lastArrivalTime_ == WallTime(0) || fps_ < 0) {
      // called first time
      lastArrivalTime_  = arrivalTime;
      cameraTime_       = arrivalTime;
      if (*currentFrameTime == WallTime(0)) {
        *currentFrameTime  = arrivalTime;
      }
      lastFrameCount_ = frameCount;
      return (dtAvg);
    }
    // Guard against wrap of frame numbers, in a very crude way.
    // At 40 fps, wrap around will occur after about 3 years
    // of continuous running. So in fact, this save-guard has never
    // been tested!
    if (frameCount == lastFrameCount_) {
      ROS_WARN_STREAM("camera " << id_ << " got duplicate frame: "
                      << frameCount);
    }
    int nframes = (frameCount > lastFrameCount_) ?
      (frameCount - lastFrameCount_) : 1;
    frameCount_ += nframes; // count in even the skipped ones

    // Unfortunately, the camera timestamps are not all that clean and
    // equally well spaced, either. The source of noise is not clear.
    // It could be that the trigger source is not running as
    // smoothly as one would expect, who knows.
    // To update the camera timestamps, we maintain a running average
    // of the time between frames, and use that to increment the camera time,
    // taking into account that a camera may skip a frame.

    const double dt = (arrivalTime - lastArrivalTime_).toSec() / nframes;

    // advance camera time by average time interval
    cameraTime_ = cameraTime_ + ros::WallDuration(dtAvg * nframes);
    
    // keep the old values around
    lastFrameCount_  = frameCount;
    lastArrivalTime_ = arrivalTime;

    // if a new frame started, update currentFrameTime
    if ((cameraTime_ - *currentFrameTime).toSec()  > 0.6 * dtAvg) {
      *currentFrameTime = cameraTime_;
    }
    // now bias the camera time such that it:
    //  a) drifts towards the average arrival time, i.e.
    //     if there was just mean zero noise on the arrival times,
    //     after a while the camera time would equal the
    //     expected arrival time.
    //  b) drifts towards the frameTime. As the frame time is only
    //     updated by the first camera that receives the new
    //     frame, this introduces coupling between the
    //     cameras such that they strife to maintain a common time.
    
    const double alpha_a       = 0.002;
    const double alpha_f       = 0.005;
    const double arrivalOffset = (arrivalTime       - cameraTime_).toSec();
    const double frameOffset   = (*currentFrameTime - cameraTime_).toSec();
    cameraTime_ += ros::WallDuration(arrivalOffset * alpha_a +
                                     frameOffset * alpha_f);

    // For diagnostic purposes, average the offset between the arrival
    // time and the currentFrameTime. This average must be less than
    // 1/2 of the frame time, or else the images cannot belong to the
    // same frame!
    const double alphaOffset = 0.0025;  // integration const [1/frames]
    double error = (arrivalTime - *currentFrameTime).toSec() / dtAvg;
    offset_ = offset_ * (1.0 - alphaOffset) +  alphaOffset * error;
    variance_ += (error - offset_) * (error - offset_); // good enough
    
    if (nframes != 1) {
      ROS_WARN_STREAM("camera " << id_ << " dropped frames: " << nframes - 1);
      framesDropped_ += nframes - 1;
    }
    if (debug_) {
      debugFile_ << arrivalTime - STARTUP_TIME  << " "       // 1
                 << cameraTime_ - STARTUP_TIME << " "        // 2
                 << *currentFrameTime - STARTUP_TIME << " "  // 3
                 << offset_ << " "                           // 4
                 << dt << " "                                // 5
                 << dtAvg << " "                             // 6
                 << frameCount                               // 7
                 << std::endl;
    }
    return (dt);
  }

  void CamSync::timeStampThread() {
    std::ofstream tsFile;
    lastLogTime_ = WallTime::now();
    while (ros::ok() && keepPolling_) {
      CameraFrame frame = cameraFrames_.waitForNextFrame(&keepPolling_);
      if (keepPolling_ && ros::ok()) {
        // got valid frame, send it to respective camera
        if (updateTimeStatistics(frame)) {
          frame.setMessageTimeStamp(currentFrameTime_);
          // move frame to camera thread
          cameras_[frame.cameraId]->getFrames().addFrame(frame);
        }
        const WallTime t = WallTime::now();
        if (t > lastLogTime_ + logInterval_) {
          ROS_INFO("-------------- camera stats ---------------------");
          for (const auto &cam: cameras_) {
            cam->logStats((t - lastLogTime_).toSec());
          }
          lastLogTime_ = t;
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
      WallTime grabTime = WallTime::now();
      flea3::Image pgrImage;
      bool ret = curCam->camera().GrabImage(*image_msg, &pgrImage);
#ifdef SIMULATE_FRAME_DROPS      
      if (std::rand() < failureThreshold) {
        continue;
      }
#endif      
      if (ret) {
        WallTime arrivalTime = WallTime::now();
        const FlyCapture2::ImageMetadata &metaData = pgrImage.GetMetadata();
        // put the new frame in the queue
        cameraFrames_.addFrame(camIndex, ret, grabTime, arrivalTime,
                               metaData.embeddedFrameCounter, image_msg, metaData);
        // update exposure control
        double newShutter(-1.0), newGain(-1.0);
        curCam->getExposureController()->imageCallback(
          image_msg, &newShutter, &newGain);
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
    if (masterCamIdx_ >= 0) {
      config.trigger_source  = -1; // free running
      //config.enable_output_voltage = 1; // for blackfly master
      config.exposure        = false;
      config.auto_shutter    = false;
      config.auto_gain       = false;

      CamConfig cc(config);
      auto masterCam = cameras_[masterCamIdx_];
      masterCam->Stop();
      masterCam->camera().Configure(cc);
      auto &controller = *masterCam->getExposureController();
      controller.setFPS(config_.fps, config_.max_free_fps);
      controller.setCurrentShutter(cc.shutter_ms);
      controller.setCurrentGain(cc.gain_db);
      masterCam->setFPS(fps_);
      masterCam->camera().StartCapture();
      masterCam->camera().SetEnableTimeStamps(true);
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
        auto &controller = *curCam->getExposureController();
        controller.setFPS(config_.fps, config_.max_free_fps);
        controller.setCurrentShutter(cc2.shutter_ms);
        controller.setCurrentGain(cc2.gain_db);
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

}  // namespace cam_sync
