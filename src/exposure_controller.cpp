/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer
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

#include "cam_sync/exposure_controller.h"
#include "cam_sync/MetaData.h"

//#define DEBUG

namespace cam_sync {
  ExposureController::ExposureController(const ros::NodeHandle &parentNode,
                                         const std::string &camName) {
    nh_ = ros::NodeHandle(parentNode, camName + "/exposure_control");
    name_ = camName;
    configServer_.reset(new dynamic_reconfigure::Server<ExposureControlDynConfig>(nh_));
    configServer_->setCallback(boost::bind(&ExposureController::configure, this, _1, _2));
    metaDataPub_ = nh_.advertise<MetaData>("meta_data", 1);
  }

  ExposureController::~ExposureController() {
  }

  double ExposureController::calculateGain(double brightRatio) const {
    const double kp = 5.0;  // empirical
    double desiredGain = currentGain_ + kp * log(brightRatio);
    return (std::max(std::min(desiredGain, config_.max_gain), config_.min_gain));
  }

  bool ExposureController::updateExposure(double b, double *newShutter, double *newGain) {
    double err_b = (config_.brightness - b);
    bool hasChanged(false);
#ifdef DEBUG
    ROS_INFO("b: %6.3f err_b: %5.2f curr shut: %5.2f curr gain: %5.3f wait: %1d",
             b, err_b, currentShutter_, currentGain_, numFramesSkip_);
#endif
    if (numFramesSkip_ > 0) {
      // Changes in gain or shutter take a few
      // frames to arrive at the camera, so we just skip
      // those.
      numFramesSkip_--;
      return (hasChanged);
    }

    if (fabs(err_b) > config_.brightness_tol) {
      double brightRatio  = std::max(std::min(config_.brightness/b, 10.0), 0.1);
      if (currentGain_ > config_.min_gain) {
        //
        // We are in gain control mode. Note that calculateGain()
        // can return min_gain, which switches off gain mode.
        //
        double optGain = calculateGain(brightRatio);
        if (optGain != currentGain_) {
          currentGain_ = optGain;
          *newShutter = -1; // no change to shutter
          *newGain    = currentGain_;
#ifdef DEBUG
          ROS_INFO_STREAM("changing gain to: " << currentGain_);
#endif
        }
      } else {
        //
        // We are in shutter control mode
        //
        double maxShutter = getMaxShutter();
        if (currentShutter_ >= maxShutter && err_b > 0 &&
            config_.max_gain > config_.min_gain) {
          // Already have shutter at maximum, but still not bright enough,
          // must  switch to gain control mode.
          currentGain_ = calculateGain(brightRatio);
          *newShutter = -1; // no change to shutter
          *newGain    = currentGain_;
#ifdef DEBUG
          ROS_INFO_STREAM("entering gain mode, gain: " << currentGain_);
#endif
        } else {
          // regular shutter control
          double desiredShutter = currentShutter_ * brightRatio;
          double optShutter = std::max(config_.min_shutter,
                                       std::min(desiredShutter,
                                                config_.max_shutter));
          if (optShutter != currentShutter_) {
            // we assume the driver sees the update flags
            // and will update the camera shutter to the desired value
            currentShutter_ = optShutter;
            *newShutter = currentShutter_;
            *newGain = -1.0;
#ifdef DEBUG
            //ROS_INFO_STREAM("updating shutter to: " << currentShutter_);
#endif
          }
        }
      }
    }
    hasChanged = *newGain != -1 || *newShutter != -1;
    if (hasChanged) {
      // must wait until either the time- averaged image, or
      // at the very least the camera had time to respond to the shutter
      // change.
      numFramesSkip_ = std::max(config_.wait_frames,
                                (int)(2.0 / brightnessAvgConst_));
    }
    return (hasChanged);
  }

  void ExposureController::imageCallback(const sensor_msgs::ImageConstPtr &img,
                                         double *newShutter, double *newGain) {
    *newShutter = -1;
    *newGain = -1;
    if (config_.enabled && currentShutter_ >= 0 && currentGain_ >= 0) {
      // gets the image-averaged, but instantaneous brightness: bi
      double bi = std::max(getAverageBrightness(&img->data[0], img->height,
                                               img->width, img->step), 1.0);
      // now time average
      b_ = b_ * (1.0 - brightnessAvgConst_) + brightnessAvgConst_ * bi;
      double oldShutter = currentShutter_;
      double oldGain    = currentGain_;
      if (updateExposure(b_, newShutter, newGain)) {
         ROS_INFO("%s bright: %7.2f at shut/gain: [%5.2f %5.3f] "
                  "new: [%5.2f %5.3f]", name_.c_str(), b_, oldShutter,
                  oldGain, currentShutter_, currentGain_);
      }
    }
  }

  void ExposureController::publish(const MetaData &msg) {
    if (metaDataPub_.getNumSubscribers() > 0) {
      metaDataPub_.publish(msg);
    }
  }


  double
  ExposureController::getAverageBrightness(const unsigned char *data,
                                           int rows, int cols, int stride) {
    const int DOWNSAMPLING_RATE = 32;
    if (firstRow_ < 0) {
      // should only run once after reconfiguration
      firstRow_  = (rows * config_.top_margin) / 100;
      lastRow_   = (rows * (100 - config_.bottom_margin)) / 100;
      int nlines = (lastRow_ - firstRow_ - 1) / DOWNSAMPLING_RATE + 1;
      int ncols  = (cols - 1)/ DOWNSAMPLING_RATE + 1;
      normFac_   =  1.0/(double)(nlines * ncols);
    }

    int64_t totalBrightness(0);
    int     rowByteStride(stride * DOWNSAMPLING_RATE);
    size_t  ioff(firstRow_ * stride);
    for (int i = firstRow_; i < lastRow_; i += DOWNSAMPLING_RATE) {
      for (int j = 0; j < cols; j += DOWNSAMPLING_RATE) {
        totalBrightness += data[ioff + j];
      }
      ioff += rowByteStride;
    }
    return (totalBrightness * normFac_);
  }

  void ExposureController::setFPS(double raw_fps, double maxFreeFPS) {
    
    // The frame rate at which the camera is triggered
    // determines the maximum exposure time. From PtGreys documents
    // (search for "Maximum frame rate possible in asynchronous
    // (external trigger) mode"):
    //
    // Max_Frame_Rate_Trigger = 1/(Shutter + (1/Max_Frame_Rate_Free_Running))
    //
    // or conversely: max_shutter = 1/target_fps - 1/max_fps_free
    //
    // Experiments showed that this is actually precisely the hard limit,
    // i.e. the frame rate will drop to 1/2 the moment this limit is exceeded
    // by just 1msec. For this reason the max shutter time is lowered by
    // a bit more to make sure the frame rate stays up
    
    double fps = fmax(raw_fps, 0.1);  // lower bound of 0.1 Hz
    const double headRoom = 2.0;   // additional margin of safety (msec) to avoid frame drops
    const double minShutter = 1.0; // at least one msec max shutter
    double maxShutter = fmax(1000.0/fps - 1000.0/maxFreeFPS - headRoom,
                             config_.min_shutter);
    maxShutter = std::max(maxShutter, 1.0);
    if (maxShutter != maxShutterLim_) {
      ROS_INFO("updating FPS shutter limit to %.2fms", maxShutter);
    }
    maxShutterLim_ = maxShutter;
  
    // now determine the averaging constant from the fps

    brightnessAvgConst_ = 1.0/(config_.brightness_avg_time * fps);
    
  }

  void ExposureController::configure(ExposureControlDynConfig& config,
                                     int level) {
    config_ = config;
    b_ = config.brightness; // initialize with target brightness
  }
}
