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

#pragma once

#include "ocs2_core/Types.hpp"
#include "ocs2_core/model_data/Metrics.hpp"
#include "ocs2_core/model_data/Multiplier.hpp"
#include "ocs2_core/reference/ModeSchedule.hpp"
#include "ocs2_core/reference/TargetTrajectories.hpp"
#include "ocs2_mpc/SystemObservation.hpp"
#include "ocs2_oc/oc_data/PerformanceIndex.hpp"

// MPC messages
#include "ocs2_msgs/msg/constraint.hpp"
#include "ocs2_msgs/msg/controller_data.hpp"
#include "ocs2_msgs/msg/lagrangian_metrics.hpp"
#include "ocs2_msgs/msg/mode_schedule.hpp"
#include "ocs2_msgs/msg/mpc_flattened_controller.hpp"
#include "ocs2_msgs/msg/mpc_input.hpp"
#include "ocs2_msgs/msg/mpc_observation.hpp"
#include "ocs2_msgs/msg/mpc_performance_indices.hpp"
#include "ocs2_msgs/msg/mpc_state.hpp"
#include "ocs2_msgs/msg/mpc_target_trajectories.hpp"
#include "ocs2_msgs/msg/multiplier.hpp"
#include "ocs2_msgs/srv/reset.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace ocs2
{

using ConstraintMsg = ocs2_msgs::msg::Constraint;
using ControllerDataMsg = ocs2_msgs::msg::ControllerData;
using InteractiveMarkerMsg = visualization_msgs::msg::InteractiveMarker;
using InteractiveMarkerControlMsg = visualization_msgs::msg::InteractiveMarkerControl;
using InteractiveMarkerFeedbackMsg = visualization_msgs::msg::InteractiveMarkerFeedback;
using LagrangianMetricsMsg = ocs2_msgs::msg::LagrangianMetrics;
using MarkerMsg = visualization_msgs::msg::Marker;
using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;
using ModeScheduleMsg = ocs2_msgs::msg::ModeSchedule;
using MultiplierMsg = ocs2_msgs::msg::Multiplier;
using MpcFlattenedControllerMsg = ocs2_msgs::msg::MpcFlattenedController;
using MpcInputMsg = ocs2_msgs::msg::MpcInput;
using MpcObservationMsg = ocs2_msgs::msg::MpcObservation;
using MpcPerformanceIndicesMsg = ocs2_msgs::msg::MpcPerformanceIndices;
using MpcStateMsg = ocs2_msgs::msg::MpcState;
using MpcTargetTrajectoriesMsg = ocs2_msgs::msg::MpcTargetTrajectories;
using ResetSrv = ocs2_msgs::srv::Reset;

namespace ros_msg_conversions
{

/** Creates the observation message. */
MpcObservationMsg create_observation_msg(const SystemObservation & observation);

/** Reads the observation message. */
SystemObservation read_observation_msg(const MpcObservationMsg & observation_msg);

/** Creates the mode sequence message. */
ModeScheduleMsg create_mode_schedule_msg(const ModeSchedule & mode_schedule);

/** Reads the mode sequence message. */
ModeSchedule read_mode_schedule_msg(const ModeScheduleMsg & mode_schedule_msg);

/** Creates the target trajectories message. */
MpcTargetTrajectoriesMsg create_target_trajectories_msg(
  const TargetTrajectories & target_trajectories);

/** Returns the TargetTrajectories message. */
TargetTrajectories read_target_trajectories_msg(
  const MpcTargetTrajectoriesMsg & target_trajectories_msg);

/**
 * Creates the performance indices message.
 *
 * @param [in] initTime: The initial time for which the MPC is computed.
 * @param [in] performanceIndices: The performance indices of the solver.
 * @return The performance indices ROS message.
 */
MpcPerformanceIndicesMsg create_performance_indices_msg(
  scalar_t init_time, const PerformanceIndex & performance_indices);

/** Reads the performance indices message. */
PerformanceIndex read_performance_indices_msg(
  const MpcPerformanceIndicesMsg & performance_indices_msg);

/** Creates constraint message. */
ConstraintMsg create_constraint_msg(scalar_t time, const vector_t & constraint);

/** Creates lagrangian_metrics message. */
LagrangianMetricsMsg create_lagrangian_metrics_msg(
  scalar_t time, LagrangianMetricsConstRef metrics);

/** Creates multiplier message. */
MultiplierMsg create_multiplier_msg(scalar_t time, MultiplierConstRef multiplier);

}  // namespace ros_msg_conversions
}  // namespace ocs2
