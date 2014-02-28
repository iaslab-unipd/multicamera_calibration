/*
 *  Copyright (c) 2013-2014, Filippo Basso <bassofil@dei.unipd.it>,
 *                           Riccardo Levorato <riccardo.levorato@dei.unipd.it>
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

#ifndef MULTICAMERA_CALIBRATION_ONLINE_CALIBRATION_NODE_H_
#define MULTICAMERA_CALIBRATION_ONLINE_CALIBRATION_NODE_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/String.h>

#include <calibration_common/calibration_common.h>
#include <camera_info_manager/camera_info_manager.h>

#include <calibration_common/algorithms/automatic_checkerboard_finder.h>
#include <multicamera_calibration/multicamera_calibration.h>

using namespace camera_info_manager;

namespace calibration
{

struct SensorROS
{

  typedef boost::shared_ptr<SensorROS> Ptr;
  typedef boost::shared_ptr<const SensorROS> ConstPtr;

  SensorROS(const std::string & frame_id,
            const image_transport::Subscriber & image_sub,
            const ros::Subscriber & camera_info_sub)
    : frame_id_(frame_id),
      image_sub_(image_sub),
      camera_info_sub_(camera_info_sub),
      new_image_(false)
  {
//    std::stringstream ss;
//    ss << "/sensor_" << id;
//    name_ = ss.str();
  }

  SensorROS(const std::string & frame_id)
    : frame_id_(frame_id),
      new_image_(false)
  {
    // Do nothing
  }

  std::string frame_id_;

  image_transport::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  sensor_msgs::Image::ConstPtr image_msg_;

  PinholeSensor::Ptr sensor_;

  bool new_image_;

  void imageCallback(const sensor_msgs::Image::ConstPtr & msg)
  {
    image_msg_ = msg;
    new_image_ = true;
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg)
  {
    if (not sensor_)
    {
      sensor_ = boost::make_shared<PinholeSensor>();
      sensor_->setFrameId(frame_id_);
    }

    PinholeCameraModel::ConstPtr cm = boost::make_shared<PinholeCameraModel>(*msg);
    sensor_->setCameraModel(cm);
  }

};

class OnlineCalibrationNode
{
public:

  OnlineCalibrationNode(const ros::NodeHandle & node_handle);

  void actionCallback(const std_msgs::String::ConstPtr & msg);

  bool initialize();

  void spin();

private:

  ros::NodeHandle node_handle_;
  image_transport::ImageTransport image_transport_;

  ros::Subscriber action_sub_;
  std::vector<SensorROS::Ptr> sensor_vec_;
  Checkerboard::Ptr checkerboard_;

  int num_sensors_;

  MultiCameraCalibration::Ptr calibration_;

};

} /* namespace calibration */

#endif /* MULTICAMERA_CALIBRATION_ONLINE_CALIBRATION_NODE_H_ */
