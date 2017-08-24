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
#include <ros/ros.h>



int main(int argc, char** argv) {
  ros::init(argc, argv, "cam_sync");
  ros::NodeHandle nh("~");

  try {
    float rec_length;
    nh.getParam("rec_length", rec_length);
    int num_cameras;
    nh.getParam("num_cameras", num_cameras);

    ROS_INFO("Recording for %f seconds on %d cameras.", rec_length, num_cameras);
 
    cam_sync::CamSync camsync(nh);
    camsync.start();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
