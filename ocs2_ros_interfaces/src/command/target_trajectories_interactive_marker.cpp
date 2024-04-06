/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_ros_interfaces/command/target_trajectories_interactive_marker.hpp"

#include "ocs2_ros_interfaces/common/ros_msg_conversions.hpp"

namespace ocs2
{
InteractiveMarkerMsg TargetTrajInteractiveMarker::create_interactive_marker() const
{
  InteractiveMarkerMsg marker;
  marker.header.frame_id = "world";
  marker.header.stamp = clock_interface_->get_clock()->now();
  marker.name = "Goal";
  marker.scale = 0.2;
  marker.description = "Right click to send command";
  marker.pose.position.z = 1.0;

  // create a grey box marker
  const auto boxMarker = []() {
    MarkerMsg marker;
    marker.type = MarkerMsg::CUBE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
  }();

  // create a non-interactive control which contains the box
  InteractiveMarkerControlMsg box_control;
  box_control.always_visible = 1;
  box_control.markers.push_back(boxMarker);
  box_control.interaction_mode = InteractiveMarkerControlMsg::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  marker.controls.push_back(box_control);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  InteractiveMarkerControlMsg control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControlMsg::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControlMsg::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControlMsg::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControlMsg::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControlMsg::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControlMsg::MOVE_AXIS;
  marker.controls.push_back(control);

  return marker;
}

void TargetTrajInteractiveMarker::process_feedback(
  const InteractiveMarkerFeedbackMsg::ConstSharedPtr & feedback)
{
  // Desired state trajectory
  const Eigen::Vector3d position(
    feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
  const Eigen::Quaterniond orientation(
    feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y,
    feedback->pose.orientation.z);

  // get the latest observation
  SystemObservation observation;
  {
    std::lock_guard<std::mutex> lock(latest_observation_mutex_);
    observation = latest_observation_;
  }

  const auto trajs = func_goal_pose_to_traj_(position, orientation, observation);
  traj_publisher_->publish_traj(trajs);
}
}  // namespace ocs2
