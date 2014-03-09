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

#ifndef MULTICAMERA_CALIBRATION_SIMULATION_NODE_H_
#define MULTICAMERA_CALIBRATION_SIMULATION_NODE_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/String.h>

#include <calibration_common/calibration_common.h>
#include <camera_info_manager/camera_info_manager.h>

#include <multicamera_calibration/multicamera_calibration.h>

using namespace camera_info_manager;

namespace calibration
{

namespace SimulationTypes
{
enum SimulationType
{
  LINE,
  CIRCLE
};
}
typedef SimulationTypes::SimulationType SimulationType;

class MultiCameraSimulationNode
{
public:

  MultiCameraSimulationNode(const ros::NodeHandle & node_handle);

  bool initialize();

  void spin();

private:

  void createView();

  ros::NodeHandle node_handle_;

  std::vector<PinholeSensor::Ptr> sensor_vec_;
  Checkerboard::Ptr checkerboard_;

  int num_sensors_;
  int num_samples_;
  std::string camera_calib_url_;
  std::string camera_name_;
  double image_error_;

  SimulationType type_;

  double radius_;
  double delta_radius_;
  double height_;

  double distance_;
  double min_z_, max_z_, delta_z_;

  MultiCameraCalibration::Ptr calibration_;

  std::string results_file_;

};

} /* namespace calibration */

#endif /* MULTICAMERA_CALIBRATION_SIMULATION_NODE_H_ */
