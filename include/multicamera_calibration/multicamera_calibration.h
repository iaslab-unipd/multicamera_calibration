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
#include <calibration_common/pinhole/pinhole.h>

#include <geometry_msgs/Pose.h>

using namespace camera_info_manager;

namespace calibration
{

struct SensorNode
{
  typedef boost::shared_ptr<SensorNode> Ptr;
  typedef boost::shared_ptr<const SensorNode> ConstPtr;

  static const int MAX_LEVEL;
  static const double MAX_DISTANCE;
  static const double MAX_ERROR;

  SensorNode(const PinholeSensor::Ptr & sensor,
             size_t id)
    : level_(MAX_LEVEL),
      distance_(MAX_DISTANCE),
      min_error_(MAX_ERROR),
      id_(id)
  {
    // Do nothing
  }

  PinholeSensor::Ptr sensor_;
  int level_;
  double distance_;
  double min_error_;

  size_t id_;

};

class MultiCameraCalibration
{
public:

  typedef boost::shared_ptr<MultiCameraCalibration> Ptr;
  typedef boost::shared_ptr<const MultiCameraCalibration> ConstPtr;

  MultiCameraCalibration(const ros::NodeHandle & node_handle);

  void setCheckerboard(const Checkerboard::Ptr & checkerboard)
  {
    checkerboard_ = checkerboard;
  }

  void addSensor(const PinholeSensor::Ptr & sensor)
  {
    SensorNode::Ptr sensor_node = boost::make_shared<SensorNode>(sensor, sensor_map_.size());
    sensor_map_[sensor] = sensor_node;
    sensor_vec_.push_back(sensor_node);
  }

  void nextAcquisition()
  {
    view_vec_.push_back(ViewMap());
  }

  void addData(const PinholeSensor::Ptr & sensor,
               const cv::Mat & image)
  {
    PinholeView<Checkerboard>::Ptr color_view;

    SensorNode::Ptr & sensor_node = sensor_map_[sensor];

    if (findCheckerboard(image, sensor, color_view))
      view_vec_.back()[sensor_node] = color_view;

    geometry_msgs::TransformStamped transform_msg;
    if (sensor->toTF(transform_msg))
      tf_pub_.sendTransform(transform_msg);

  }

  void perform();

  void saveTF();
  void saveTF2();
  void saveCameraAndFrames();

private:

  bool findCheckerboard(const cv::Mat & image,
                        const PinholeSensor::Ptr & sensor,
                        typename PinholeView<Checkerboard>::Ptr & color_view);

  void optimize();

  ros::NodeHandle node_handle_;

  std::map<PinholeSensor::ConstPtr, SensorNode::Ptr> sensor_map_;
  std::vector<SensorNode::Ptr> sensor_vec_;
  Checkerboard::Ptr checkerboard_;

  AutomaticCheckerboardFinder finder_;

  BaseObject::Ptr world_;

  tf::TransformListener tf_listener_;

  tf::TransformBroadcaster tf_pub_;
  ros::Publisher marker_pub_;

  bool world_set_;

  typedef std::map<SensorNode::Ptr, PinholeView<Checkerboard>::Ptr> ViewMap;
  std::vector<ViewMap> view_vec_;

  bool initialization_;
  int last_optimization_;

};

} /* namespace calibration */

#endif /* MULTICAMERA_CALIBRATION_MULTICAMERA_CALIBRATION_H_ */
