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

#include <multicamera_calibration/online_calibration_node.h>

#include <fstream>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <ceres/ceres.h>

#define OPTIMIZATION_COUNT 10

namespace calibration
{

OnlineCalibrationNode::OnlineCalibrationNode(const ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    image_transport_(node_handle)
{
  action_sub_ = node_handle_.subscribe("action", 1, &OnlineCalibrationNode::actionCallback, this);

  node_handle_.param("num_cameras", num_sensors_, 0); // TODO change parameter name

  double cell_width, cell_height;
  int rows, cols;

  bool cb_ok = true;
  cb_ok = cb_ok and node_handle_.getParam("cell_width", cell_width);
  cb_ok = cb_ok and node_handle_.getParam("cell_height", cell_height);
  cb_ok = cb_ok and node_handle_.getParam("rows", rows);
  cb_ok = cb_ok and node_handle_.getParam("cols", cols);
  if (not cb_ok)
    ROS_FATAL("Checkerboard parameter missing! Please set \"rows\", \"cols\", \"cell_width\" and \"cell_height\".");

  checkerboard_ = boost::make_shared<Checkerboard>(rows, cols, cell_width, cell_height);
  checkerboard_->setFrameId("/checkerboard");

  for (int id = 0; id < num_sensors_; ++id)
  {
    std::stringstream ss;
    ss << "/sensor_" << id;
    SensorROS::Ptr sensor = boost::make_shared<SensorROS>(ss.str());

    ss.str("");
    ss << "sensor_" << id << "/image";
    sensor->image_sub_ = image_transport_.subscribe(ss.str(), 1, &SensorROS::imageCallback, sensor);

    ss.str("");
    ss << "sensor_" << id << "/camera_info";
    sensor->camera_info_sub_ = node_handle_.subscribe<sensor_msgs::CameraInfo>(ss.str(),
                                                                               1,
                                                                               &SensorROS::cameraInfoCallback,
                                                                               sensor);

    std::string frame_id;
    ss.str("");
    ss << "sensor_" << id << "/name";
    if (node_handle_.getParam(ss.str(), frame_id))
      sensor->frame_id_ = frame_id;

    sensor_vec_.push_back(sensor);
  }

}

bool OnlineCalibrationNode::initialize()
{
  calibration_ = boost::make_shared<MultiCameraCalibration>(node_handle_);
  calibration_->setCheckerboard(checkerboard_);

  for (size_t i = 0; i < sensor_vec_.size(); ++i)
    calibration_->addSensor(sensor_vec_[i]->sensor_);

  return true;
}

void OnlineCalibrationNode::actionCallback(const std_msgs::String::ConstPtr & msg)
{
  if (msg->data == "save")
  {
    calibration_->saveTF();
  }
  else if (msg->data == "saveCam2WorldPose")
  {
    calibration_->saveCameraAndFrames();
  }
}

void OnlineCalibrationNode::spin()
{
  ros::Rate rate(2.0);

  while (ros::ok())
  {
    ros::spinOnce();

    size_t n = calibration_->nextAcquisition();

    for (int i = 0; i < num_sensors_; ++i)
    {
      const SensorROS::Ptr & sensor = sensor_vec_[i];

      if (not sensor->image_msg_)
        continue;
      try
      {
        if (sensor->new_image_)
        {
          cv_bridge::CvImage::Ptr image_ptr = cv_bridge::toCvCopy(sensor->image_msg_,
                                                                  sensor_msgs::image_encodings::BGR8);
          calibration_->addData(sensor->sensor_, image_ptr->image);
          sensor->new_image_ = false;

          std::stringstream image_file_name;
          image_file_name << "/tmp/cam" << i << "_image"<< std::setw(4) << std::setfill('0') << n << ".png";
          cv::imwrite(image_file_name.str(), image_ptr->image);

        }
      }
      catch (cv_bridge::Exception & ex)
      {
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        return;
      }
    }

    calibration_->perform();

    rate.sleep();
  }

}

} /* namespace calibration */

using namespace calibration;

int main(int argc,
         char ** argv)
{
  ros::init(argc, argv, "multicamera_calibration");
  ros::NodeHandle node_handle("~");

  try
  {
    OnlineCalibrationNode calib_node(node_handle);
    if (not calib_node.initialize())
      return 0;
    calib_node.spin();
  }
  catch (std::runtime_error & error)
  {
    ROS_FATAL("Calibration error: %s", error.what());
    return 1;
  }

  return 0;
}
