/*
 *  Copyright (c) 2013-2014, Filippo Basso <bassofil@dei.unipd.it>
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

#include <multicamera_calibration/simulation_node.h>

#include <calibration_common/pinhole/pinhole.h>
#include <camera_info_manager/camera_info_manager.h>

using namespace camera_info_manager;

namespace calibration
{

MultiCameraSimulationNode::MultiCameraSimulationNode(const ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    num_sensors_(0)
{
  if (not node_handle_.getParam("camera_calib_url", camera_calib_url_))
    ROS_FATAL("Missing \"camera_calib_url\" parameter!!");

  node_handle_.param("camera_name", camera_name_, std::string("camera"));

  node_handle_.param("num_sensors", num_sensors_, 0);

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

  node_handle_.param("image_error", image_error_, 0.5);

  std::string type_string;
  node_handle_.param("type", type_string, std::string("line"));

  if (type_string == "line")
  {
    type_ = SimulationTypes::LINE;
    node_handle_.param("distance", distance_, 1.0);
    node_handle_.param("min_z", min_z_, 1.0);
    node_handle_.param("max_z", max_z_, 2.0);
    node_handle_.param("delta_z", delta_z_, 0.5);
  }
  else if (type_string == "circle")
  {
    type_ = SimulationTypes::CIRCLE;
    node_handle_.param("radius", radius_, 1.0);
    node_handle_.param("delta_radius", delta_radius_, 0.5);
    node_handle_.param("height", height_, 3.0);
  }
  else
    ROS_FATAL("\"type\" must be either \"line\" or \"circle\"");

  node_handle_.param("num_samples", num_samples_, num_sensors_ * 10);

  node_handle_.param("results_file", results_file_, std::string("/tmp/results.txt"));

}

bool MultiCameraSimulationNode::initialize()
{
  CameraInfoManager manager(node_handle_, camera_name_, camera_calib_url_);
  PinholeCameraModel::ConstPtr pinhole_model = boost::make_shared<PinholeCameraModel>(manager.getCameraInfo());

  for (int i = 0; i < num_sensors_; ++i)
  {
    PinholeSensor::Ptr sensor = boost::make_shared<PinholeSensor>();

    std::stringstream ss;
    ss << "/sensor_" << i;
    sensor->setFrameId(ss.str());
    sensor->setCameraModel(pinhole_model);

    sensor_vec_.push_back(sensor);
  }

  calibration_ = boost::make_shared<MultiCameraCalibration>(node_handle_);
  calibration_->setCheckerboard(checkerboard_);

  for (size_t i = 0; i < sensor_vec_.size(); ++i)
    calibration_->addSensor(sensor_vec_[i]);

  return true;
}

void MultiCameraSimulationNode::spin()
{
  boost::mt19937 random_gen(time(0));

  boost::normal_distribution<> image_error(0.0, image_error_);
  boost::variate_generator<boost::mt19937 &, boost::normal_distribution<> > image_noise(random_gen, image_error);

  std::vector<PinholeSensor::Ptr> sensor_vec_gt;
  BaseObject::Ptr world = boost::make_shared<BaseObject>();
  world->setFrameId("/world");

  if (type_ == SimulationTypes::LINE)
  {
    for (size_t i = 0; i < sensor_vec_.size(); ++i)
    {
      PinholeSensor::Ptr sensor = boost::make_shared<PinholeSensor>(*sensor_vec_[i]);
      if (i > 0)
        sensor->transform(Transform::Identity() * Translation3(i * distance_, 0.0, 0.0));
      sensor->setParent(world);
      sensor_vec_gt.push_back(sensor);
    }

    ros::Rate rate(30.0);

    for (double z = min_z_; z < max_z_ + 0.01; z += delta_z_)
    {
      double delta_x = ((distance_ * (num_sensors_ - 1)) + z) / num_samples_;
      for (double x = -z / 2.0; x < (distance_ * (num_sensors_ - 1)) + z / 2.0; x += delta_x)
      {
        Transform T = Transform::Identity();
        T *= Translation3(x, 0.0, z) * Translation3(-checkerboard_->center());

        Checkerboard::Ptr real_cb = boost::make_shared<Checkerboard>(*checkerboard_);
        real_cb->transform(T);

        calibration_->nextAcquisition();

        for (size_t i = 0; i < sensor_vec_.size(); ++i)
        {
          Checkerboard::Ptr viewed_cb = boost::make_shared<Checkerboard>(*real_cb);
          viewed_cb->transform(sensor_vec_gt[i]->pose().inverse());

          // Create image

          cv::Mat image(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
          Cloud2 image_corners = sensor_vec_gt[i]->cameraModel()->project3dToPixel(viewed_cb->corners());

          std::vector<cv::Point2f> corners;
          for (size_t c = 0; c < image_corners.size(); ++c)
          {
            if (image_corners[c].x() > 0 and image_corners[c].x() < image.cols and image_corners[c].y() > 0
                and image_corners[c].y() < image.rows)
              corners.push_back(cv::Point2f(image_corners[c].x(), image_corners[c].y()));

            image_corners[c].x() += image_noise();
            image_corners[c].y() += image_noise();

          }
          //          cv::drawChessboardCorners(image,
          //                                    cv::Size(image_corners.xSize(), image_corners.ySize()),
          //                                    corners,
          //                                    corners.size() == image_corners.size());
          //
          //          cv::imshow(sensor_vec_gt[i]->frameId(), image);
          //          cv::waitKey(100);

          if (corners.size() == image_corners.size())
          {
            PinholeView<Checkerboard>::Ptr color_view = boost::make_shared<PinholeView<Checkerboard> >();
            color_view->setId(sensor_vec_[i]->frameId());
            color_view->setObject(checkerboard_);
            color_view->setPoints(image_corners);
            color_view->setSensor(sensor_vec_[i]);

            calibration_->addData(sensor_vec_[i], color_view);
          }

        }

        calibration_->perform();

        rate.sleep();

      }
    }

    calibration_->optimize();

    std::ofstream file(results_file_.c_str());
    file << "num_sensors: " << num_sensors_ << std::endl;
    file << "distance: " << distance_ << std::endl;
    file << "min_z: " << min_z_ << std::endl;
    file << "max_z: " << max_z_ << std::endl;
    file << "delta_z: " << delta_z_ << std::endl;
    file << "num_samples: " << num_samples_ << " x " << int((max_z_ - min_z_) / delta_z_) + 1 << std::endl << std::endl;

    for (size_t i = 0; i < sensor_vec_.size(); ++i)
    {
      PinholeSensor::Ptr sensor = sensor_vec_[i];

      Pose pose = sensor->pose();
      BaseObject::ConstPtr parent = sensor->parent();
      while (parent->parent())
      {
        pose = parent->pose() * pose;
        sensor->setPose(pose);
        parent = parent->parent();
        sensor->setParent(parent);
      }

      geometry_msgs::TransformStamped transform_msg;
      if (sensor->toTF(transform_msg))
      {
        file << transform_msg.header.frame_id << " -> " << transform_msg.child_frame_id << std::endl;
        file << transform_msg.transform << std::endl;
      }
    }

    file.close();

  }
  else // (type_ == SimulationTypes::CIRCLE)
  {

    double rot_angle = 2 * M_PI / sensor_vec_.size();
    double camera_angle = std::atan2(height_, radius_);

    for (size_t i = 0; i < sensor_vec_.size(); ++i)
    {
      PinholeSensor::Ptr sensor = boost::make_shared<PinholeSensor>(*sensor_vec_[i]);
      sensor->transform(Transform::Identity() * AngleAxis(-camera_angle, Vector3::UnitX()));
      sensor->transform(Transform::Identity() * Translation3(0.0, 0.0, -radius_));
      if (i > 0)
        sensor->transform(Transform::Identity() * AngleAxis(rot_angle * i, Vector3::UnitY()));
      sensor->setParent(world);
      sensor_vec_gt.push_back(sensor);
    }

    ros::Rate rate(30.0);

    for (double z = 0.0; z < radius_ + 0.01; z += delta_radius_)//double a = 0.0; a < M_PI / 2 + 0.01; a += M_PI
    {
      double delta_a = 2 * M_PI / num_samples_;
      for (double a = 0.0; a < 2 * M_PI; a += delta_a)
      {
        Transform T = Transform::Identity();
        T = Translation3(-checkerboard_->center());
        T = AngleAxis(camera_angle, Vector3::UnitX()) * T;
        T = AngleAxis(a, Vector3::UnitY()) * Translation3(0.0, height_, -z) * T;

        Checkerboard::Ptr real_cb = boost::make_shared<Checkerboard>(*checkerboard_);
        real_cb->transform(T);

        calibration_->nextAcquisition();

        for (size_t i = 0; i < sensor_vec_.size(); ++i)
        {
          Checkerboard::Ptr viewed_cb = boost::make_shared<Checkerboard>(*real_cb);
          viewed_cb->transform(sensor_vec_gt[i]->pose().inverse());

          viewed_cb->plane().normal().dot(Vector3::UnitZ());

          // Create image

          cv::Mat image(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
          Cloud2 image_corners = sensor_vec_gt[i]->cameraModel()->project3dToPixel(viewed_cb->corners());

          std::vector<cv::Point2f> corners;
          for (size_t c = 0; c < image_corners.size(); ++c)
          {
            if (image_corners[c].x() > 0 and image_corners[c].x() < image.cols and image_corners[c].y() > 0
                and image_corners[c].y() < image.rows)
              corners.push_back(cv::Point2f(image_corners[c].x(), image_corners[c].y()));

            image_corners[c].x() += image_noise();
            image_corners[c].y() += image_noise();

          }
          cv::drawChessboardCorners(image,
                                    cv::Size(image_corners.xSize(), image_corners.ySize()),
                                    corners,
                                    corners.size() == image_corners.size());

          cv::imshow(sensor_vec_gt[i]->frameId(), image);
          cv::waitKey(100);

          if (corners.size() == image_corners.size())
          {
            PinholeView<Checkerboard>::Ptr color_view = boost::make_shared<PinholeView<Checkerboard> >();
            color_view->setId(sensor_vec_[i]->frameId());
            color_view->setObject(checkerboard_);
            color_view->setPoints(image_corners);
            color_view->setSensor(sensor_vec_[i]);

            calibration_->addData(sensor_vec_[i], color_view);
          }

        }

        calibration_->perform();

        rate.sleep();

      }
    }
  }

  ROS_INFO("Calibration complete!");

  ros::Rate rate(10.0);
  while (ros::ok())
  {
    calibration_->publish();
    rate.sleep();
  }

}

} /* namespace calibration */

int main(int argc,
         char ** argv)
{
  ros::init(argc, argv, "simulation");
  ros::NodeHandle node_handle("~");

  try
  {
    calibration::MultiCameraSimulationNode sim_node(node_handle);
    if (not sim_node.initialize())
      return 0;
    sim_node.spin();
  }
  catch (std::runtime_error & error)
  {
    ROS_FATAL("Calibration error: %s", error.what());
    return 1;
  }

  return 0;
}
