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

#ifndef CAMSYNC_EXPOSURE_CONTROLLER_H
#define CAMSYNC_EXPOSURE_CONTROLLER_H

#include "cam_sync/ExposureControlDynConfig.h"
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <memory>
#include <sensor_msgs/Image.h>

namespace cam_sync {
  class ExposureController {
  public:
    ExposureController(const std::string &camName);
    ~ExposureController();
    ExposureController(const ExposureController&) = delete;
    ExposureController& operator=(const ExposureController&) = delete;
    void imageCallback(const sensor_msgs::ImageConstPtr &img,
                       double *newShutter, double *newGain);

    void setFPS(double fps, double maxFreeFPS);
    void setCurrentShutter(double ms) { currentShutter_ = ms; }
    void setCurrentGain(double gain)  { currentGain_ = gain; }
    void configure(ExposureControlDynConfig &config, int level);
    double  getMaxShutter() const { return std::min(config_.max_shutter, maxShutterLim_);}
  private:
    double  calculateGain(double brightRatio) const;
    double  getAverageBrightness(const unsigned char *data,
                                 int rows, int cols, int stride);
    // ----------- variables
    ros::NodeHandle           nh_;
    ExposureControlDynConfig  config_;
    double                    fps_{15.0};
    int                       numFramesSkip_{0};   // num frames to skip
    double                    currentShutter_{-1}; // in msec
    double                    currentGain_{-1};    // in db
    double                    normFac_{0};
    double                    maxShutterLim_{100}; // in msec, limit due to frame rate
    int                       firstRow_{-1};
    int                       lastRow_{-1};
    std::shared_ptr<dynamic_reconfigure::Server<ExposureControlDynConfig> >    configServer_;
  };

}

#endif
