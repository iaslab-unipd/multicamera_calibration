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

#include <multicamera_calibration/multicamera_calibration.h>

#include <fstream>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <ceres/ceres.h>

#define OPTIMIZATION_COUNT 10

namespace calibration
{

const int SensorNode::MAX_LEVEL = 10000;
const double SensorNode::MAX_DISTANCE = 10000.0;
const double SensorNode::MAX_ERROR = 10000.0;

MultiCameraCalibration::MultiCameraCalibration(const ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    world_(boost::make_shared<BaseObject>()),
    world_set_(false),
    initialization_(true),
    last_optimization_(0)
{
  world_->setFrameId("/world");
  marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("markers", 0);
}

bool MultiCameraCalibration::findCheckerboard(const cv::Mat & image,
                                              const PinholeSensor::Ptr & sensor,
                                              typename PinholeView<Checkerboard>::Ptr & color_view)
{
  const SensorNode::Ptr & sensor_node = sensor_map_[sensor];

  Cloud2 corners(checkerboard_->rows(), checkerboard_->cols());
  finder_.setImage(image);
  if (finder_.find(*checkerboard_, corners))
  {
    color_view = boost::make_shared<PinholeView<Checkerboard> >();
    color_view->setId(sensor->frameId());
    color_view->setObject(checkerboard_);
    color_view->setPoints(corners);
    color_view->setSensor(sensor);

    Checkerboard checkerboard(*color_view);

    visualization_msgs::Marker checkerboard_marker;
    checkerboard_marker.ns = "checkerboard";
    checkerboard_marker.id = sensor_node->id_;

    checkerboard.toMarker(checkerboard_marker);
    marker_pub_.publish(checkerboard_marker);

    geometry_msgs::TransformStamped transform_msg;
    checkerboard.toTF(transform_msg);
    tf_pub_.sendTransform(transform_msg);

    return true;
  }
  return false;
}

void MultiCameraCalibration::perform()
{
  const ViewMap & view_map = view_vec_.back();

  if (initialization_)
  {

    if (not world_set_ and not view_map.empty()) // Set /world
    {
      SensorNode::Ptr sensor_node = view_map.begin()->first;
      sensor_node->sensor_->setParent(world_);
      sensor_node->level_ = 0;
      world_set_ = true;
    }

    if (view_map.size() < 2)
    {
      view_vec_.resize(view_vec_.size() - 1); // Remove data
    }
    else // At least 2 cameras
    {
      int min_level = SensorNode::MAX_LEVEL;
      SensorNode::Ptr min_sensor_node;
      for (ViewMap::const_iterator it = view_map.begin(); it != view_map.end(); ++it)
      {
        SensorNode::Ptr sensor_node = it->first;
        if (sensor_node->level_ < min_level)
        {
          min_level = sensor_node->level_;
          min_sensor_node = sensor_node;
        }
      }

      if (min_level < SensorNode::MAX_LEVEL) // At least one already in tree
      {
        for (ViewMap::const_iterator it = view_map.begin(); it != view_map.end(); ++it)
        {
          SensorNode::Ptr sensor_node = it->first;
          if (sensor_node != min_sensor_node)
          {
            Checkerboard checkerboard(*it->second);
            double camera_error = std::abs(checkerboard.plane().normal().dot(Vector3::UnitZ())) * checkerboard.center().squaredNorm();

            if (sensor_node->level_ > min_level + 1 and camera_error < sensor_node->min_error_)
            {
              Checkerboard min_checkerboard(*view_map.at(min_sensor_node));

              double distance = (min_checkerboard.center() - checkerboard.center()).norm();

              sensor_node->sensor_->setParent(min_sensor_node->sensor_);
              sensor_node->sensor_->setPose(min_checkerboard.pose() * checkerboard.pose().inverse());

              sensor_node->min_error_ = camera_error;
              sensor_node->distance_ = distance;
              sensor_node->level_ = min_level + 1;
            }
          }
        }

      }

      initialization_ = (view_vec_.size() < 20);
      for (int i = 0; i < sensor_vec_.size(); ++i)
      {
        SensorNode::Ptr & sensor_node = sensor_vec_[i];
        if (sensor_node->level_ == SensorNode::MAX_LEVEL)
        {
          initialization_ = true;
          break;
        }
      }

    }
  }
  else
  {

    if (view_map.size() < 2)
    {
      view_vec_.resize(view_vec_.size() - 1); // Remove data
    }
    else // At least 2 cameras
    {
      if (last_optimization_ == 0)
      {
        optimize();
        last_optimization_ = OPTIMIZATION_COUNT;
      }
      else
        last_optimization_--;
    }

  }

  publish();

}

void MultiCameraCalibration::publish()
{

  for (size_t i = 0; i < sensor_vec_.size(); ++i)
  {
    SensorNode::Ptr sensor_node = sensor_vec_[i];
    geometry_msgs::TransformStamped transform_msg;
    if (sensor_node->sensor_->toTF(transform_msg))
      tf_pub_.sendTransform(transform_msg);
  }

}

class CheckerboardError
{
public:

  CheckerboardError(const PinholeCameraModel::ConstPtr & camera_model,
                    const Checkerboard::ConstPtr & checkerboard,
                    const Cloud2 & image_corners)
    : camera_model_(camera_model),
      checkerboard_(checkerboard),
      image_corners_(image_corners)
  {
  }

  template <typename T>
    bool operator ()(const T * const checkerboard_pose,
                     T * residuals) const
    {

      typename Types<T>::Vector3 checkerboard_r_vec(checkerboard_pose[0], checkerboard_pose[1], checkerboard_pose[2]);
      typename Types<T>::AngleAxis checkerboard_r(checkerboard_r_vec.norm(), checkerboard_r_vec.normalized());
      typename Types<T>::Translation3 checkerboard_t(checkerboard_pose[3], checkerboard_pose[4], checkerboard_pose[5]);

      typename Types<T>::Transform checkerboard_pose_eigen = checkerboard_t * checkerboard_r;

      typename Types<T>::Cloud3 cb_corners(checkerboard_->cols(), checkerboard_->rows());
      cb_corners.matrix() = checkerboard_pose_eigen * checkerboard_->corners().matrix().cast<T>();

      typename Types<T>::Cloud2 reprojected_corners = camera_model_->project3dToPixel<T>(cb_corners);

      for (size_t i = 0; i < cb_corners.size(); ++i)
        residuals[i] = T((reprojected_corners[i] - image_corners_[i].cast<T>()).norm());

      return true;
    }

private:

  const PinholeCameraModel::ConstPtr & camera_model_;
  const Checkerboard::ConstPtr & checkerboard_;
  const Cloud2 & image_corners_;

};

class GlobalError
{
public:

  GlobalError(const PinholeCameraModel::ConstPtr & camera_model,
              const Checkerboard::ConstPtr & checkerboard,
              const Cloud2 & image_corners)
    : camera_model_(camera_model),
      checkerboard_(checkerboard),
      image_corners_(image_corners)
  {
  }

  template <typename T>
    bool operator ()(const T * const sensor_pose,
                     const T * const checkerboard_pose,
                     T * residuals) const
    {
      typename Types<T>::Vector3 sensor_r_vec(sensor_pose[0], sensor_pose[1], sensor_pose[2]);
      typename Types<T>::AngleAxis sensor_r(sensor_r_vec.norm(), sensor_r_vec.normalized());
      typename Types<T>::Translation3 sensor_t(sensor_pose[3], sensor_pose[4], sensor_pose[5]);

      typename Types<T>::Transform sensor_pose_eigen = Types<T>::Transform::Identity() * sensor_t;

      if (sensor_r_vec.norm() != T(0))
        sensor_pose_eigen = sensor_t * sensor_r;

      typename Types<T>::Vector3 checkerboard_r_vec(checkerboard_pose[0], checkerboard_pose[1], checkerboard_pose[2]);
      typename Types<T>::AngleAxis checkerboard_r(checkerboard_r_vec.norm(), checkerboard_r_vec.normalized());
      typename Types<T>::Translation3 checkerboard_t(checkerboard_pose[3], checkerboard_pose[4], checkerboard_pose[5]);

      typename Types<T>::Transform checkerboard_pose_eigen = checkerboard_t * checkerboard_r;

      typename Types<T>::Cloud3 cb_corners(checkerboard_->cols(), checkerboard_->rows());
      cb_corners.matrix() = sensor_pose_eigen.inverse() * checkerboard_pose_eigen
                            * checkerboard_->corners().matrix().cast<T>();

      typename Types<T>::Cloud2 reprojected_corners = camera_model_->project3dToPixel<T>(cb_corners);

      for (size_t i = 0; i < cb_corners.size(); ++i)
        residuals[i] = T((reprojected_corners[i] - image_corners_[i].cast<T>()).norm());

      return true;
    }

private:

  const PinholeCameraModel::ConstPtr & camera_model_;
  const Checkerboard::ConstPtr & checkerboard_;
  const Cloud2 & image_corners_;

};

void MultiCameraCalibration::optimize()
{

  ROS_INFO("Optimizing...");

  ceres::Problem problem;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 6, Eigen::DontAlign | Eigen::RowMajor> cb_data(view_vec_.size(), 6);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 6, Eigen::DontAlign | Eigen::RowMajor> sensor_data(sensor_vec_.size(), 6);

  for (size_t i = 0; i < sensor_vec_.size(); ++i)
  {
    const SensorNode & sensor_node = *sensor_vec_[i];
    Pose pose = sensor_node.sensor_->pose();
    BaseObject::ConstPtr parent = sensor_node.sensor_->parent();
//    std::cout << i << ": " << parent->frameId() << " " << pose.translation().transpose() << std::endl;
    while (parent->parent())
    {
      pose = parent->pose() * pose; //TODO controllare
      parent = parent->parent();
      sensor_node.sensor_->setParent(parent);
      sensor_node.sensor_->setPose(pose);
//      std::cout << parent->frameId() << " " << pose.translation().transpose() << std::endl;
    }

    AngleAxis rotation = AngleAxis(pose.linear());
    sensor_data.row(i).head<3>() = rotation.angle() * rotation.axis();
    sensor_data.row(i).tail<3>() = pose.translation();
  }

//  ROS_INFO("Before optimization:");
//  for (size_t i = 0; i < sensor_vec_.size(); ++i)
//  {
//    ROS_INFO_STREAM("(" << sensor_vec_[i]->sensor_->parent()->frameId() << ") " << sensor_vec_[i]->sensor_->frameId() << ": " << sensor_data.row(i));
//  }

  for (size_t i = 0; i < view_vec_.size(); ++i)
  {
    ViewMap & data_map = view_vec_[i];
    ViewMap::iterator it = data_map.begin(); //TODO prendere la migliore e non la prima
    const PinholeView<Checkerboard>::Ptr & view = it->second;
    const SensorNode::Ptr & sensor_node = it->first;
    Pose pose = sensor_node->sensor_->cameraModel()->estimatePose(view->points(), view->object()->points());
    BaseObject::ConstPtr parent = sensor_node->sensor_;
    while (parent)
    {
      pose = parent->pose() * pose; //TODO controllare
      parent = parent->parent();
    }

    AngleAxis rotation = AngleAxis(pose.linear());
    cb_data.row(i).head<3>() = rotation.angle() * rotation.axis();
    cb_data.row(i).tail<3>() = pose.translation();

    for (ViewMap::iterator it = data_map.begin(); it != data_map.end(); ++it)
    {
      PinholeView<Checkerboard>::Ptr & view = it->second;
      const SensorNode::Ptr & sensor_node = it->first;

      if (sensor_node->level_ > 0)
      {
        GlobalError * error = new GlobalError(sensor_node->sensor_->cameraModel(), view->object(), view->points());

        typedef ceres::AutoDiffCostFunction<GlobalError, ceres::DYNAMIC, 6, 6> GlobalErrorFunction;

        ceres::CostFunction * cost_function = new GlobalErrorFunction(error, checkerboard_->size());
        problem.AddResidualBlock(cost_function, NULL, sensor_data.row(sensor_node->id_).data(), cb_data.row(i).data());
      }
      else
      {
        CheckerboardError * error = new CheckerboardError(sensor_node->sensor_->cameraModel(),
                                                          view->object(),
                                                          view->points());

        typedef ceres::AutoDiffCostFunction<CheckerboardError, ceres::DYNAMIC, 6> CheckerboardErrorFunction;

        ceres::CostFunction * cost_function = new CheckerboardErrorFunction(error, checkerboard_->size());
        problem.AddResidualBlock(cost_function, NULL, cb_data.row(i).data());
      }

    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.max_num_iterations = 50;
//  options.minimizer_progress_to_stdout = true;
  options.num_threads = 8;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

//  ROS_INFO("After optimization:");
  for (size_t i = 0; i < sensor_vec_.size(); ++i)
  {
//    ROS_INFO_STREAM("(" << sensor_vec_[i]->sensor_->parent()->frameId() << ") " << sensor_vec_[i]->sensor_->frameId() << ": " << sensor_data.row(i));
    SensorNode::Ptr & sensor_node = sensor_vec_[i];
    if (sensor_data.row(i).head<3>().norm() != 0)
    {
      AngleAxis rotation(sensor_data.row(i).head<3>().norm(), sensor_data.row(i).head<3>().normalized());
      Translation3 translation(sensor_data.row(i).tail<3>());
      sensor_node->sensor_->setPose(translation * rotation);
    }
    else
    {
      Translation3 translation(sensor_data.row(i).tail<3>());
      sensor_node->sensor_->setPose(Pose::Identity() * translation);
    }
  }

//  ROS_INFO("--------------------------------------------------------------------------------");
}

void MultiCameraCalibration::saveTF2()
{
//save tf between camera and world coordinate system ( chessboard ) to set_transformations.launch file
  std::string file_name = ros::package::getPath("multicamera_calibration") + "/launch/frames.launch";
  std::ofstream launch_file;
  launch_file.open(file_name.c_str());
  if (launch_file.is_open())
  {
    launch_file << "<launch>" << std::endl << std::endl;

    tf::StampedTransform transform, transform_final;

    launch_file << "<!-- Default parameters-->" << std::endl << "<arg name=\"asus2_name\" default=\"asus2\" />"
                << std::endl << "<arg name=\"asus1_name\" default=\"asus1\" />" << std::endl
                << "<arg name=\"kinect_id\" value=\"A00367A01433047A\" />" << std::endl
                << "<arg name=\"period\" default=\"10\" />" << std::endl << std::endl;

    for (size_t id = 0; id < sensor_vec_.size(); ++id)
    {
      //const Pose & pose = camera_vector_[id].sensor_->pose();
      //const Pose& pose = checkerboard_->
      //Quaternion q(pose.linear());

      std::stringstream ss, frame;
      ss << id;
      frame << "_depth_optical_frame";

      tf_listener_.lookupTransform("checkerboard_" + ss.str(),
                                   sensor_vec_[id]->sensor_->frameId(),
                                   ros::Time(0),
                                   transform);
      //tf_listener.lookUpTransform("checkerboard_"+id, camera_vector_[id].sensor_->parent()->frameId(), ros::Time(0), transform);

      //ROS_INFO("Trasformata fra %20s e %20s: ", "checkerboard_"+ss.str(), camera_vector_[id].sensor_->frameId());

      Eigen::Affine3d rotate_depth_frame;

      /*
       * 			rotate_depth_frame.linear() <<   0, 0, 1,
       -1, 0,-1,
       0,-1, 0;
       */
      /*			rotate_depth_frame.linear() <<  0,-1, 0,
       0, 0,-1,
       1, 0, 0;
       */

      /*
       ROS_INFO("X: %f Y: %f Z: %f",transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

       Eigen::Affine3d final, transform_eigen;
       tf::transformTFToEigen(transform, transform_eigen);
       transform_eigen = rotate_depth_frame * transform_eigen;
       tf::transformEigenToTF(transform_eigen, transform_final);

       ROS_INFO("X: %f Y: %f Z: %f",transform_final.getOrigin().x(), transform_final.getOrigin().y(), transform_final.getOrigin().z());
       */

      transform_final = transform;

      launch_file << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\""
                  << sensor_vec_[id]->sensor_->frameId().substr(1) << "_broadcaster\" args=\""
                  << transform_final.getOrigin().x() << " " << transform_final.getOrigin().y() << " "
                  << transform_final.getOrigin().z() << " " << transform_final.getRotation().x() << " "
                  << transform_final.getRotation().y() << " " << transform_final.getRotation().z() << " "
                  << "checkerboard_" << id << " " << sensor_vec_[id]->sensor_->frameId() << " 100\" />\n\n";

    }

    launch_file
      << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"asus1_broadcaster2\" args=\"0 0 0 0 0 0 1 /asus1 /asus1_link  100\" />"
      << std::endl
      << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"asus2_broadcaster2\" args=\"0 0 0 0 0 0 1 /asus2 /asus2_link  100\" />"
      << std::endl << std::endl;

    launch_file << "<!-- Launching asus --> " << std::endl
                << "<include file=\"$(find openni_launch)/launch/openni.launch\">" << std::endl
                << "\t<arg name=\"camera\" value=\"$(arg asus1_name)\" />" << std::endl
                << "\t<!--arg name=\"publish_tf\" value=\"false\" /-->" << std::endl << "</include>" << std::endl
                << std::endl;

    launch_file << "<!-- Openning Rviz for visualization-->" << std::endl
                << "<node name=\"rviz\" pkg=\"rviz\" type=\"rviz\" args=\"-d $(env HOME)/.rviz/multikinect.rviz\"/>"
                << std::endl;

    launch_file << "</launch>" << std::endl;
  }
  launch_file.close();
  ROS_INFO_STREAM(file_name << " created!");
}

void MultiCameraCalibration::saveCameraAndFrames()
{
  for (size_t id = 0; id < sensor_vec_.size(); ++id)
  {
    if (sensor_vec_[id]->level_ == SensorNode::MAX_LEVEL)
    {
      ROS_WARN("Not all camera poses estimated!!! File not saved!!!");
      return;
    }
  }

//save tf between camera and world coordinate system ( chessboard ) to set_transformations.launch file
  std::string file_name = ros::package::getPath("multicamera_calibration") + "/launch/cameras_and_frames.launch";
  std::ofstream launch_file;
  launch_file.open(file_name.c_str());

  std::stringstream optical_frame_string, link_string;

  optical_frame_string << "_rgb_optical_frame";
  link_string << "_link";

  if (launch_file.is_open())
  {
    launch_file << "<launch>" << std::endl << std::endl;

    tf::StampedTransform transform, link_transform;
    tf::Transform transform_final;

    launch_file << "<!-- Default parameters-->" << std::endl << "<arg name=\"asus2_name\" default=\"asus2\" />"
                << std::endl << " <arg name=\"asus1_name\" default=\"asus1\" />" << std::endl
                << " <arg name=\"kinect_id\" value=\"A00367A01433047A\" />" << std::endl
                << " <arg name=\"period\" default=\"10\" />" << std::endl << std::endl;

    for (size_t id = 0; id < sensor_vec_.size(); ++id)
    {
      //const Pose & pose = camera_vector_[id].sensor_->pose();
      //const Pose& pose = checkerboard_->
      //Quaternion q(pose.linear());

      std::stringstream ss;
      ss << id;

      //tf_listener.lookUpTransform("checkerboard_"+id, camera_vector_[id].sensor_->parent()->frameId(), ros::Time(0), transform);

      //ROS_INFO("Trasformata fra %20s e %20s: ", "checkerboard_"+ss.str(), camera_vector_[id].sensor_->frameId());

      Eigen::Affine3d rotate_depth_frame;

      /*
       * 			rotate_depth_frame.linear() <<   0, 0, 1,
       -1, 0,-1,
       0,-1, 0;
       */
      /*			rotate_depth_frame.linear() <<  0,-1, 0,
       0, 0,-1,
       1, 0, 0;
       */

      /*
       ROS_INFO("X: %f Y: %f Z: %f",transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

       Eigen::Affine3d final, transform_eigen;
       tf::transformTFToEigen(transform, transform_eigen);
       transform_eigen = rotate_depth_frame * transform_eigen;
       tf::transformEigenToTF(transform_eigen, transform_final);

       ROS_INFO("X: %f Y: %f Z: %f",transform_final.getOrigin().x(), transform_final.getOrigin().y(), transform_final.getOrigin().z());
       */

      //CERCO LA POSIZIONE RELATIVA FRA OPTICAL FRAME E SCACCHIERA
      tf_listener_.lookupTransform("checkerboard_" + ss.str(),
                                   sensor_vec_[id]->sensor_->frameId() + (optical_frame_string.str()),
                                   ros::Time(0),
                                   transform);

      tf::Matrix3x3 temp_rotation;
      temp_rotation.setRPY(M_PI, 0, -M_PI / 2);
      tf::Vector3 temp_origin = tf::Vector3(0.0, 0.0, 0.0);
      tf::Transform flip_z = tf::Transform(temp_rotation, temp_origin);

      //(LEGACY)
      //transform_final = transform;
      transform_final = flip_z * tf::Transform(transform);

      double yaw, pitch, roll;
      tf::Matrix3x3(transform_final.getRotation()).getRPY(roll, pitch, yaw);

      //transform_final.getOrigin().Vector3().

      //DICHIARO LA POSIZIONE RELATIVA FRA CAMERA E SCACCHIERA
      launch_file << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\""
                  << sensor_vec_[id]->sensor_->frameId().substr(1) << "_broadcaster\" args=\""
                  << transform_final.getOrigin().x() << " " << transform_final.getOrigin().y() << " "
                  << transform_final.getOrigin().z() << " " << yaw << " " << pitch << " " << roll << " " << "world"
                  << " " << sensor_vec_[id]->sensor_->frameId() << " 100\" />\n\n";

      //CERCO LA POSIZIONE RELATIVA FRA GLI ALBERI: ASUS_FRAME E ASUSX_LINK ( CHE DOVREBBE ESERE ZERO )
      tf_listener_.lookupTransform(sensor_vec_[id]->sensor_->frameId() + (optical_frame_string.str()),
                                   sensor_vec_[id]->sensor_->frameId() + (link_string.str()),
                                   ros::Time(0),
                                   link_transform);

      //DICHIARO LA POSIZIONE RELATIVA FRA
      launch_file << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\""
                  << sensor_vec_[id]->sensor_->frameId().substr(1) << "_broadcaster2\" args=\" -0.045 0 0 "
                  //<< link_transform.getRotation().x() << " " << link_transform.getRotation().y() << " " << link_transform.getRotation().z() << " "
                  << "1.57 -1.57 0 " << sensor_vec_[id]->sensor_->frameId() << " "
                  << sensor_vec_[id]->sensor_->frameId() + link_string.str() << " 100\" />\n\n";
    }

    /*
     Eigen::Affine3d rotate_depth_frame;

     rotate_depth_frame.linear() <<  0,-1, 0,
     0, 0,-1,
     1, 0, 0;

     Eigen::Affine3d transform_eigen;
     tf::transformTFToEigen(transform, transform_eigen);
     transform_eigen = rotate_depth_frame * transform_eigen;
     tf::transformEigenToTF(transform_eigen, link_transform);
     */

    /*
     tfListener.lookupTransform(camera_vector_[id].sensor_->frameId()+(link_string.str()), camera_vector_[id].sensor_->frameId()+(optical_frame_string.str()), ros::Time(0), link_transform);


     launch_file <<  "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"asus1_broadcaster2\" args=\"0 0 0 " << link_transform.getRotation().x() << " " << link_transform.getRotation().y() << " " << link_transform.getRotation().z() << " /asus1 /asus1_link  100\" />" << std::endl
     <<	"<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"asus2_broadcaster2\" args=\"0 0 0 " << link_transform.getRotation().x() << " " << link_transform.getRotation().y() << " " << link_transform.getRotation().z() << " /asus2 /asus2_link  100\" />" << std::endl << std::endl;
     */

    launch_file << "<!-- Launching asus --> " << std::endl
                << "<include file=\"$(find openni_launch)/launch/openni.launch\">" << std::endl
                << "<arg name=\"camera\" value=\"$(arg asus1_name)\" />" << std::endl
                << "<!--arg name=\"publish_tf\" value=\"false\" /-->" << std::endl << "</include>" << std::endl
                << std::endl;

//		launch_file <<  "<!-- Openning Rviz for visualization-->" << std::endl
//					<<	"<node name=\"rviz\" pkg=\"rviz\" type=\"rviz\" args=\"-d $(env HOME)/.rviz/multikinect.rviz\"/>" << std::endl << std::endl;

    launch_file << "</launch>" << std::endl;
  }
  launch_file.close();
  ROS_INFO_STREAM(file_name << " created!");
}

void MultiCameraCalibration::saveTF()
{
  for (size_t i = 0; i < sensor_vec_.size(); ++i)
  {
    if (sensor_vec_[i]->level_ == SensorNode::MAX_LEVEL)
    {
      ROS_WARN("Not all camera poses estimated!!! File not saved!!!");
      return;
    }
  }

//save tf to set_transformations.launch file
  std::string file_name = ros::package::getPath("multicamera_calibration") + "/launch/frames.launch";
  std::ofstream launch_file;
  launch_file.open(file_name.c_str());
  if (launch_file.is_open())
  {
    launch_file << "<launch>" << std::endl << std::endl;

    for (size_t i = 0; i < sensor_vec_.size(); ++i)
    {
      const Pose & pose = sensor_vec_[i]->sensor_->pose();
      Quaternion q(pose.linear());
      launch_file << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\""
      << sensor_vec_[i]->sensor_->frameId().substr(1) << "_broadcaster\" args=\"" << pose.translation().transpose()
      << " " << q.coeffs().transpose() << " " << sensor_vec_[i]->sensor_->parent()->frameId() << " "
      << sensor_vec_[i]->sensor_->frameId() << " 100\" />\n\n";
    }
    launch_file << "</launch>" << std::endl;
  }
  launch_file.close();
  ROS_INFO_STREAM(file_name << " created!");
}

} /* namespace calibration */
