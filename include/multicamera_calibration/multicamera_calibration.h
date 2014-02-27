/*
 *  Copyright (c) 2013-2014, Filippo Basso <bassofil@dei.unipd.it>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MULTICAMERA_CALIBRATION_MULTICAMERA_CALIBRATION_H_
#define MULTICAMERA_CALIBRATION_MULTICAMERA_CALIBRATION_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <calibration_common/calibration_common.h>
#include <camera_info_manager/camera_info_manager.h>

#include <calibration_common/algorithms/automatic_checkerboard_finder.h>
#include <calibration_common/pinhole/camera_model.h>

#include <geometry_msgs/Pose.h>

using namespace camera_info_manager;

namespace calibration
{

const int MAX_LEVEL = 10000;
const double MAX_DISTANCE = 10000.0;
const double MAX_ERROR = 10000.0;

struct Camera
{

  Camera(int id,
         const image_transport::Subscriber & image_sub,
         const ros::Subscriber & camera_info_sub)
    : id_(id),
      image_sub_(image_sub),
      camera_info_sub_(camera_info_sub),
      level_(MAX_LEVEL),
      distance_(MAX_DISTANCE),
      min_error_(MAX_ERROR)
  {
    std::stringstream ss;
    ss << "/camera_" << id;
    name_ = ss.str();
  }

  Camera(int id)
    : id_(id),
      level_(MAX_LEVEL),
      distance_(MAX_DISTANCE),
      min_error_(MAX_ERROR)
  {
    std::stringstream ss;
    ss << "/camera_" << id;
    name_ = ss.str();
  }

  int id_;
  std::string name_;

  image_transport::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;

  PinholeSensor::Ptr sensor_;
  int level_;
  double distance_;

  double min_error_;

  sensor_msgs::Image::ConstPtr image_msg_;

};

class MultiCameraCalibration
{
public:

  MultiCameraCalibration(ros::NodeHandle & node_handle);

  void imageCallback(const sensor_msgs::Image::ConstPtr & msg,
                     int id);

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg,
                          int id);

  void actionCallback(const std_msgs::String::ConstPtr & msg);

  bool initialize();

  void spin();

private:

  bool findCheckerboard(cv::Mat & image,
                        int id,
                        typename PinholeView<Checkerboard>::Ptr & color_view);

  void optimize();

  void saveTF();
  void saveTF2();
  void saveCameraAndFrames();

  ros::NodeHandle node_handle_;
  image_transport::ImageTransport image_transport_;

  ros::Subscriber action_sub_;
  std::vector<Camera> camera_vec_;
  Checkerboard::Ptr checkerboard_;

  AutomaticCheckerboardFinder finder_;

  int num_cameras_;

  BaseObject::Ptr world_;

  tf::TransformListener tfListener;

  tf::TransformBroadcaster tf_pub_;
  ros::Publisher marker_pub_;

  bool world_set_;

  typedef std::map<int, PinholeView<Checkerboard>::Ptr> DataMap;
  std::vector<DataMap> data_vec_;

  bool initialization_;
  bool stop_;

};

} /* namespace calibration */

#endif /* MULTICAMERA_CALIBRATION_MULTICAMERA_CALIBRATION_H_ */
