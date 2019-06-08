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
#include <exception>
#include <thread>
#include <chrono>


//#define SIMULATE_FRAME_DROPS


namespace cam_sync {
  using boost::irange;

  static ros::WallTime STARTUP_TIME = ros::WallTime::now();
  
  static double get_pgr_timestamp(const flea3::Image &pgrImage) {
    flea3::TimeStamp ts = pgrImage.GetTimeStamp();
    const double CYCLE_COUNT_TO_SEC = 0.000125; // 8kHz
    const double CYCLE_OFFSET_TO_SEC= CYCLE_COUNT_TO_SEC / 4096; // 12 bit
    double tstamp = ts.cycleSeconds + ts.cycleCount * CYCLE_COUNT_TO_SEC +
      ts.cycleOffset * CYCLE_OFFSET_TO_SEC;
    return (tstamp);
  }
  
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
                    int i, int lagThresh, double att,
                    bool debug, const std::string& prefix) :
    Flea3Ros(pnh, prefix), id_(i), frames_(i),
    lagThreshold_(lagThresh), arrivalToCameraTimeCoeff_(att), debug_(debug) {
    exposureController_.reset(new ExposureController(pnh, prefix));
    // metadata is in useless relative numbers, need to convert to
    // absolute
    shutterRatio_    = camera().GetAbsToRelativeRatio(0x918, 0x81c);
    // for the gain, we have to set it to something > 0 to compute a ratio!
    bool autoGain(false);  double gain(1.0);
    camera().SetGain(autoGain, gain);
    // now find the gain ratio
    gainRatio_ = camera().GetAbsToRelativeRatio(0x928, 0x820);
    if (gainRatio_ < 0) {
      gainRatio_ = 0.1; // fallback to some reasonable value
      ROS_WARN_STREAM("cam" << i << " cannot find gain ratio!");
    }
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
    msg.shutter       = (float)(md.embeddedShutter & 0x00000FFF)*shutterRatio_;
    msg.gain          = (float)(md.embeddedGain   & 0x00000FFF) *gainRatio_;
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

  void CamSync::Cam::setFPS(double f, double freqTol,
                            double minDelay, double maxDelay, double window) {
    fps_ = f;
    SetTopicDiagnosticParameters(
      fps_ * (1.0 - freqTol), fps_ * (1.0 + freqTol),
      window, minDelay / fps_, maxDelay / fps_);
  }
  
  bool
  CamSync::Cam::isWarmedUp(const WallTime &t) const {
    return (t > firstArrivalTime_ + WallDuration(3.0));
  }

  ros::WallTime
  CamSync::GlobalTime::findFrameTime(const WallTime &cameraTime,
                                     const double dtAvg) {
    WallTime frameTime(0);
    // step through the known frame times, starting with the most recent
    // and going towards the oldest in the list
    for (auto it = frameTimes_.begin(); it != frameTimes_.end(); ++it) {
      double tdiff = (cameraTime - *it).toSec();
      if (tdiff > 0.6 * dtAvg) {
        // This must be a newer frame, insert it into list.
        // Rather than using camera time, advance the previous frame time
        double adj = getAvg();
        resetOffset(); // reset average offset
        // new frame time is the old time, advanced by adjustment that
        // brings frame time into line with average time stamp
        ros::WallTime newFrameTime = *it +
          WallDuration(adj + dtAvg * std::round(tdiff/dtAvg));
        frameTimes_.insert(it, newFrameTime); // goes in before(!) iterator
        frameTime = newFrameTime;
        break;
      }
      if (std::abs(tdiff) <= 0.6 * dtAvg) {
        // matches an existing frame
        frameTime = *it;
        recordOffset(tdiff); // add offset to averaging statistic
        break;
      }
    }

    // if we have not inserted it anywhere, it must be at the end of
    // the list of time frames, or the list of frames is empty.
 
    if (frameTime == WallTime(0)) {
      if (!frameTimes_.empty()) { // not found in list, and list not empty!
        // cameraTime is so old that we have already cleared the list
        auto late = (frameTimes_.back() - cameraTime).toSec() / dtAvg;
        ROS_WARN_STREAM("got very old frame that is late by "
                        << late << " frames");
      }
      frameTime = cameraTime; // establish a new frame time
      frameTimes_.push_back(cameraTime);
    }
    // 
    if (frameTimes_.size() * dtAvg > 0.5) { // limit list size to 0.5sec
      frameTimes_.pop_back();
    }
    return (frameTime);
  }

  void
  CamSync::Cam::initializeTimeStats(const CameraFrame &f) {
    lastArrivalTime_  = f.arrivalTime;
    firstArrivalTime_ = f.arrivalTime;
    cameraTime_       = f.arrivalTime;
    lastFrameTime_    = f.arrivalTime;
    imageTime_        = f.arrivalTime;
    lastFrameCount_   = f.metaData.embeddedFrameCounter;
    lastCameraClock_  = f.cameraClock;
  }

  void CamSync::Cam::updateImageTime(const CameraFramePtr &fp) {
    // wrap around, see point grey documents
    if (lastCameraClock_ > fp->cameraClock) lastCameraClock_ -= 128.0;
    // now compute correct time delta
    const double dt   = fp->cameraClock - lastCameraClock_;
    lastCameraClock_ = fp->cameraClock;
    fp->imageTime = imageTime_ + WallDuration(dt);
  }

  /*
   * gotNewFrame() updates some statistics, but does not yet attempt
   * to align camera time with global frame time.
   */
  double
  CamSync::Cam::gotNewFrame(const CameraFrame &f,
                            double dtAvg, int *nframes) {
    const double dt = (f.arrivalTime - lastArrivalTime_).toSec();
    
    // Guard against wrap of frame numbers, in a very crude way.
    // At 40 fps, wrap around will occur after about 3 years
    // of continuous running. So in fact, this save-guard has never
    // been tested!
    const unsigned int cameraFrameCount = f.metaData.embeddedFrameCounter;
    if (cameraFrameCount == lastFrameCount_) {
      ROS_WARN_STREAM("camera " << id_ << " got duplicate frame: "
                      << cameraFrameCount);
    }

#if USE_FRAME_COUNT_ONLY
    *nframes = (cameraFrameCount > lastFrameCount_) ?
      (cameraFrameCount - lastFrameCount_) : 1;
#else
    const double dtImg = (f.imageTime - imageTime_).toSec();
    *nframes = std::max(1, (int)std::round(dtImg/dtAvg));
#endif
    imageTime_ = f.imageTime;

    frameCount_ += *nframes; // count in even the skipped ones
    
    lastFrameCount_  = cameraFrameCount;
    if (*nframes != 1) {
      ROS_WARN_STREAM("camera " << id_ << ": dropped frames: " << *nframes-1);
      framesDropped_ += *nframes - 1;
    }
    lastArrivalTime_ = f.arrivalTime;
    return (dt / *nframes);
  }
  
  /*
   * updateCameraTime() primarily maintains a per-camera clock.
   * This is done by using the frame numbers that are embedded
   * in the image, and the wall clock arrival times of the images.
   * The global frame time is updated here as well, by whichever camera
   * gets the new frame first.
   */
  bool
  CamSync::Cam::updateCameraTime(const CameraFrame &f,
                                 int nframes, double dtAvg,
                                 WallTime *frameTime,
                                 GlobalTime *globalTime) {
    const WallTime &arrivalTime = f.arrivalTime;
    bool gotValidFrame(true);
    // Unfortunately, the point grey camera timestamps are not all
    // that clean and equally well spaced, so we don't use them.
    // The source of noise is not clear. It could be that the trigger
    // source is not running as smoothly as one would expect, who knows.
    //
    // To update the camera time, we maintain a running average
    // of the time between frames, and use that to increment the camera time,
    // taking into account that a camera may skip a frame.

    // advance camera time by average time interval
    const WallDuration dtn(dtAvg * nframes);
    cameraTime_ = cameraTime_ + dtn;

    if (arrivalTime - cameraTime_ > WallDuration(0.8 * dtAvg)) {
      // camera time is lagging behind
      if (++lagCounter_ > lagThreshold_) {
        lagCounter_ = 0;
        ROS_WARN_STREAM("cam " << id_ << " skipped a frame: " << arrivalTime
                        << " " << arrivalTime - STARTUP_TIME);
        cameraTime_ = cameraTime_ + dtn;
      }
    } else {
      // camera time has caught up, never mind!
      lagCounter_ = 0;
    }
    
    // look in a global list if another camera has already
    // established a frame time for this frame. If not, create a new one.

    *frameTime = globalTime->findFrameTime(cameraTime_, dtAvg);
    if (*frameTime == lastFrameTime_) {
      ROS_WARN_STREAM("duplicate frame time cam" << id_ << " t: "
                       << *frameTime << " t: " << *frameTime - STARTUP_TIME);
      gotValidFrame = false;
    }
    lastFrameTime_ = *frameTime;
    
    // now bias the camera time such that it drifts towards the
    // average arrival time, i.e. if there was just mean zero noise
    // on the arrival times, after a while the camera time would equal the
    // expected arrival time.
    
    // alpha_a speed [1/frames] of converg. of cam time to avg arrival time

    const double arrivalOffset = (arrivalTime  - cameraTime_).toSec();
    cameraTime_ += WallDuration(arrivalOffset * arrivalToCameraTimeCoeff_);

    // For diagnostic purposes, average the offset between the arrival
    // time and the frameTime. This average must be less than
    // 1/2 of the frame time, or else the images cannot belong to the
    // same frame!
    const double alphaOffset = 0.025;  // integration const [1/frames]
    double error = (arrivalTime - *frameTime).toSec() / dtAvg;
    offset_ = offset_ * (1.0 - alphaOffset) +  alphaOffset * error;
    variance_ += (error - offset_) * (error - offset_); // good enough
    
    if (debug_) {
      debugFile_ << arrivalTime - STARTUP_TIME  << " "       // 1
                 << f.imageTime - STARTUP_TIME  << " "       // 2
                 << cameraTime_ - STARTUP_TIME << " "        // 3
                 << *frameTime  - STARTUP_TIME << " "        // 4
                 << offset_ << " "                           // 5
                 << dtAvg << " "                             // 6
                 << nframes                                  // 7
                 << std::endl;
    }
    return (gotValidFrame);
  }

  CamSync::CameraFramePtr
  CamSync::FrameQueue::waitForNextFrame(bool *keepRunning) {
    CameraFramePtr frame;
    std::unique_lock<std::mutex> lock(mutex);
    const std::chrono::microseconds timeout((int64_t)(10000000LL));
    while (ros::ok() && *keepRunning && frames.empty()) {
      cv.wait_for(lock, timeout);
    }
    if (!frames.empty()) {
      frame = frames.back();
      frames.pop_back();
      if (frames.size() > 50) {
        // Ouch, we have built up a queue. This will consume a lot
        // of memory!
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

  void CamSync::FrameQueue::addFrame(const CameraFramePtr &frame) {
    std::unique_lock<std::mutex> lock(mutex);
    frames.push_front(frame);
    cv.notify_all();
  }

  CamSync::CamSync(const ros::NodeHandle& pn)
    : nh_(pn) {
    configServer_.reset(new dynamic_reconfigure::Server<Config>(nh_));
    
    pn.getParam("num_cameras", numCameras_);
    pn.param<int>("master_camera_index", masterCamIdx_, 0);
    pn.param<bool>("rotate_image", rotateImage_, false);
    pn.param<bool>("debug_timestamps", debugTimeStamps_, false);
    double logInterval;
    pn.param<double>("logging_interval", logInterval, 60.0);
    logInterval_ = WallDuration(logInterval);
    double fps(30.0);
    pn.getParam("fps", fps);
    setFPS(fps);
    double dtAvgTime;
    pn.param<double>("dt_avg_time", dtAvgTime, 10.0);
    dtAvgConst_ = 1.0 / (dtAvgTime * fps * numCameras_);
    double dtAvgNoiseThresh;
    pn.param<double>("dt_avg_noise_threshold", dtAvgNoiseThresh, 1e-2);
    dtVarThreshold_ = dtAvgNoiseThresh * dtAvgNoiseThresh;
    int lagThresh;
    pn.param<int>("lag_threshold", lagThresh, 10);
    double atctc;
    pn.param<double>("arrival_to_camera_time_coeff", atctc, 0.005);
    std::string calibDir;
    pn.param<std::string>("calib_dir", calibDir, "camera_info");
    dt_    = 1.0 / fps;  // starting estimate for average
    dtVar_ = dtVarThreshold_ * 5.0;
    for (int i = 0; i < numCameras_; i++) {
      std::string camName = "cam" + std::to_string(i);
      std::string calibfn = camName + ".yaml";
      pn.param<std::string>(camName + "/calib_file_name", calibfn, camName);
      if (!pn.hasParam(camName + "/camera_name")) {
        ROS_ERROR_STREAM("parameters missing for " << camName);
        throw (std::runtime_error("no param found for " + camName));
      }
      pn.setParam(camName + "/calib_url", calibDir + "/" + calibfn);
      CamPtr camTmp = boost::make_shared<Cam>(nh_, i, lagThresh, atctc,
                                              debugTimeStamps_, camName);
      cameras_.push_back(move(camTmp));
    }
    configServer_->setCallback(boost::bind(&CamSync::configure, this, _1, _2));
  }

  CamSync::~CamSync() {
    stopPoll();
  }

  void CamSync::setFPS(double fps) {
    fps_ = fps;
  }


  void CamSync::start() {
    double duration(60);  // in seconds
    nh_.getParam("rec_length", duration);

    timer_ = nh_.createTimer(ros::Duration(duration),
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
    if (level < 0) {
      ROS_INFO("%s: %s", nh_.getNamespace().c_str(),
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
      auto frame = curCam->getFrames().waitForNextFrame(&keepPolling_);
      if (!frame->isValid) {
        continue;
      }
      if (curCam->getNumSubscribers() > 0) {
        if (rotateImage_) {
          curCam->publishMsg(rotate_image(frame->msg), frame->metaData);
        } else {
          curCam->publishMsg(frame->msg, frame->metaData);
        }
      }
    }
    ROS_INFO_STREAM("frame publish thread done!");
  }

  bool CamSync::updateTimeStatistics(const CameraFramePtr &fp,
                                     WallTime *frameTime) {
    const CameraFrame &frame = *fp;
    bool gotValidFrame(false);
    const auto &cam = cameras_[frame.cameraId];
    if (!cam->timeStatsInitialized()) {
      cam->initializeTimeStats(frame);
      return (false);
    }
    int nframes;
    cam->updateImageTime(fp);
    const double dt = cam->gotNewFrame(frame, dt_, &nframes);
    if (!cam->isWarmedUp(frame.arrivalTime)) {
      // During the first few frames the dt from the camera are
      // very noisy, so ignore them
      return (false);
    }
    
    // some course outlier filtering before the dt goes into
    // the average
    const double lowLim(0.3 * dt_), upLim(1.7 * dt_);
    const double dtc = (dt < lowLim) ? lowLim : ((dt > upLim) ? upLim : dt);

    const double dtBefore = dt_; // remember so we can do some statistics
    
    // keep exponential average
    dt_ = dt_ * (1.0 - dtAvgConst_) + dtAvgConst_ * dtc;

    if (!warmedUp_) {
      // dtErr = how much change there was in the average
      const double dtErr = (dtBefore - dt_)/dt_;
      // now keep average of difference to the mean, similar to variance
      dtVar_ = dtVar_ * (1.0 - dtAvgConst_) + dtAvgConst_ * dtErr * dtErr;
      if (dtVar_ < dtVarThreshold_) {
        ROS_INFO("Warmed up!");
        warmedUp_ = true;
      } else {
        ROS_INFO_THROTTLE(2, "warming up, progress on variance: %.2lf%%",
                          dtVarThreshold_ / dtVar_ * 100);
      }
      // reset cameraTime to arrival Time
      cam->setCameraTime(frame.arrivalTime);
    } else {
      gotValidFrame = cam->updateCameraTime(frame, nframes,
                                            dt_, frameTime, &globalTime_);
    }
    return (gotValidFrame);
  }

  void CamSync::softwareTriggerThread() {
    ros::Rate trigger_rate(fps_);
    while (ros::ok() && keepPolling_) {
      for (const auto &cam: cameras_) {
        // Should we use RequestSingle() or FireSofwareTrigger()?
        // RequestSingle() first waits for the
        // trigger to be available, so that will hold up triggering
        // of the other cameras.
        //cam->camera().RequestSingle();
        cam->camera().FireSoftwareTrigger();
        // Firing the cameras in parallel with separate threads
        // did not show any improvement. Likely the driver is
        // serializing on the FireSoftwareTrigger().
      }
      trigger_rate.sleep();
    }
  }

  void CamSync::timeStampThread() {
    std::ofstream tsFile;
    lastLogTime_ = WallTime::now();
    while (ros::ok() && keepPolling_) {
      CameraFramePtr frame = cameraFrames_.waitForNextFrame(&keepPolling_);
      if (keepPolling_ && ros::ok()) {
        WallTime frameTime;
        if (updateTimeStatistics(frame, &frameTime)) {
          // got valid frame, dispatch it to respective camera
          frame->setMessageTimeStamp(frameTime);
          // move frame to camera thread
          cameras_[frame->cameraId]->getFrames().addFrame(frame);
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
        // adjust arrival time for delay due to different shutters!
        const FlyCapture2::ImageMetadata &md = pgrImage.GetMetadata();
        const WallTime arrTime = WallTime::now()
          - WallDuration((md.embeddedShutter & 0x00000FFF) *
                         curCam->getShutterRatio());
        const double imgTime = get_pgr_timestamp(pgrImage);
        CameraFramePtr fp(new CameraFrame(camIndex, ret, grabTime, arrTime,
                                          imgTime, md, image_msg));
        cameraFrames_.addFrame(fp);

        // update exposure control
        double newShutter(-1.0), newGain(-1.0);
        curCam->getExposureController()->imageCallback(
          image_msg, &newShutter, &newGain);
        setShutter(curCam, newShutter);
        setGain(curCam, newGain);
      } else {
        ROS_WARN_STREAM("camera " << camIndex << ": frame grab failed!");
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

    for (int i = 0; i < numCameras_; i++) {
      CamPtr cam = cameras_[i];
      CamConfig cc(config); // deep copy
      double freqTol, minDelay, maxDelay, window;
      const std::string prefix("cam" + std::to_string(i) + "/white_balance");
      nh_.param<int>(prefix + "/red",  cc.wb_red,  cc.wb_red);
      nh_.param<int>(prefix + "/blue", cc.wb_blue, cc.wb_blue);
      const std::string diag("cam" + std::to_string(i) + "/diagnostics");
      nh_.param<double>(diag + "/frequency_tolerance", freqTol, 0.02);
      nh_.param<double>(diag + "/min_delay", minDelay, -1.0);
      nh_.param<double>(diag + "/max_delay", maxDelay,  2.0);
      nh_.param<double>(diag + "/window",  window,  10.0);
      nh_.param<int>(diag + "/trigger_source",  cc.trigger_source,  cc.trigger_source);
      if (i == masterCamIdx_) {
        cc.trigger_source = -1; // free running
        cc.enable_output_voltage = 0; // once used for blackfly
      } else {
        cc.fps = fps_ * 1.5; // not sure that makes any difference...
        cc.enable_output_voltage = 0; // not needed for slaves
        cc.strobe_control = -1;  // no strobe control for slave
        cc.trigger_mode   = 14;  // overlapped processing
      }
      cam->Stop();
      ROS_INFO_STREAM("configuring cam " << i << " trigger mode: "<< cc.trigger_mode);
      cam->camera().Configure(cc);
      cam->camera().SetEnableTimeStamps(true);
      auto &controller = *cam->getExposureController();
      controller.setFPS(config_.fps, config_.max_free_fps);
      controller.setCurrentShutter(cc.shutter_ms);
      controller.setCurrentGain(cc.gain_db);
      cam->setFPS(fps_, freqTol, minDelay, maxDelay, window);
      cam->camera().StartCapture();
    }
  }

  bool CamSync::startPoll() {
    std::unique_lock<std::mutex> lock(pollMutex_);
    if (frameGrabThreads_.empty()) {
      keepPolling_ = true;
      for (int i = 0; i < numCameras_; ++i) {
        cameras_[i]->camera().StartCapture();
        frameGrabThreads_.push_back(
          boost::make_shared<boost::thread>(&CamSync::frameGrabThread,
                                            this, i));
        framePublishThreads_.push_back(
          boost::make_shared<boost::thread>(&CamSync::framePublishThread,
                                            this, i));
      }
      if (!timeStampThread_) {
        timeStampThread_ =
          boost::make_shared<boost::thread>(&CamSync::timeStampThread, this);
      }
      if (config_.use_software_trigger &&
          !softwareTriggerThread_) {
        softwareTriggerThread_ =
          boost::make_shared<boost::thread>(
            &CamSync::softwareTriggerThread, this);
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
      if (softwareTriggerThread_) {
        softwareTriggerThread_->join();
        softwareTriggerThread_.reset();
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
