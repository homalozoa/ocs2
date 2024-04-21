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

#include "ocs2_ros_interfaces/common/ros_msg_conversions.hpp"

namespace ocs2
{
namespace ros_msg_conversions
{

MpcObservationMsg create_observation_msg(const SystemObservation & observation)
{
  MpcObservationMsg observation_msg;

  observation_msg.time = observation.time;
  observation_msg.state.value.resize(observation.state.rows());
  for (size_t i = 0; i < observation.state.rows(); i++) {
    observation_msg.state.value[i] = static_cast<float>(observation.state(i));
  }
  observation_msg.input.value.resize(observation.input.rows());
  for (size_t i = 0; i < observation.input.rows(); i++) {
    observation_msg.input.value[i] = static_cast<float>(observation.input(i));
  }
  observation_msg.mode = observation.mode;

  return observation_msg;
}

SystemObservation read_observation_msg(const MpcObservationMsg & observationMsg)
{
  SystemObservation observation;

  observation.time = observationMsg.time;
  const auto & state = observationMsg.state.value;
  observation.state =
    Eigen::Map<const Eigen::VectorXf>(state.data(), state.size()).cast<scalar_t>();
  const auto & input = observationMsg.input.value;
  observation.input =
    Eigen::Map<const Eigen::VectorXf>(input.data(), input.size()).cast<scalar_t>();
  observation.mode = observationMsg.mode;

  return observation;
}

ModeScheduleMsg create_mode_schedule_msg(const ModeSchedule & mode_schedule)
{
  ModeScheduleMsg mode_schedule_msg;

  mode_schedule_msg.event_times.clear();
  mode_schedule_msg.event_times.reserve(mode_schedule.event_times.size());
  for (const auto & ti : mode_schedule.event_times) {
    mode_schedule_msg.event_times.push_back(ti);
  }
  mode_schedule_msg.mode_sequence.clear();
  mode_schedule_msg.mode_sequence.reserve(mode_schedule.mode_sequence.size());
  for (const auto & si : mode_schedule.mode_sequence) {
    mode_schedule_msg.mode_sequence.push_back(si);
  }

  return mode_schedule_msg;
}

ModeSchedule read_mode_schedule_msg(const ModeScheduleMsg & mode_schedule_msg)
{
  ModeSchedule mode_schedule;

  mode_schedule.event_times.reserve(mode_schedule_msg.event_times.size());
  for (const auto & ti : mode_schedule_msg.event_times) {
    mode_schedule.event_times.push_back(ti);
  }
  mode_schedule.mode_sequence.reserve(mode_schedule_msg.mode_sequence.size());
  for (const auto & si : mode_schedule_msg.mode_sequence) {
    mode_schedule.mode_sequence.push_back(si);
  }

  return mode_schedule;
}

MpcPerformanceIndicesMsg create_performance_indices_msg(
  scalar_t init_time, const PerformanceIndex & performance_indices)
{
  MpcPerformanceIndicesMsg performance_indices_msg;

  performance_indices_msg.init_time = init_time;
  performance_indices_msg.merit = performance_indices.merit;
  performance_indices_msg.cost = performance_indices.cost;
  performance_indices_msg.dynamics_violation_sse = performance_indices.dynamicsViolationSSE;
  performance_indices_msg.equality_constraints_sse = performance_indices.equalityConstraintsSSE;
  performance_indices_msg.equality_lagrangian = performance_indices.equalityLagrangian;
  performance_indices_msg.inequality_lagrangian = performance_indices.inequalityLagrangian;

  return performance_indices_msg;
}

PerformanceIndex read_performance_indices_msg(
  const MpcPerformanceIndicesMsg & performance_indices_msg)
{
  PerformanceIndex performance_indices;

  performance_indices.merit = performance_indices_msg.merit;
  performance_indices.cost = performance_indices_msg.cost;
  performance_indices.dynamicsViolationSSE = performance_indices_msg.dynamics_violation_sse;
  performance_indices.equalityConstraintsSSE = performance_indices_msg.equality_constraints_sse;
  performance_indices.equalityLagrangian = performance_indices_msg.equality_lagrangian;
  performance_indices.inequalityLagrangian = performance_indices_msg.inequality_lagrangian;

  return performance_indices;
}

MpcTargetTrajectoriesMsg create_target_trajectories_msg(
  const TargetTrajectories & target_trajectories)
{
  MpcTargetTrajectoriesMsg target_trajectories_msg;
  const auto & time_trajectory = target_trajectories.timeTrajectory;
  const auto & state_trajectory = target_trajectories.stateTrajectory;
  const auto & input_trajectory = target_trajectories.inputTrajectory;

  size_t N = state_trajectory.size();
  target_trajectories_msg.time_trajectory.resize(N);
  target_trajectories_msg.state_trajectory.resize(N);
  for (size_t i = 0; i < N; i++) {
    target_trajectories_msg.time_trajectory[i] = time_trajectory[i];
    target_trajectories_msg.state_trajectory[i].value = std::vector<float>(
      state_trajectory[i].data(), state_trajectory[i].data() + state_trajectory[i].size());
  }

  N = input_trajectory.size();
  target_trajectories_msg.input_trajectory.resize(N);
  for (size_t i = 0; i < N; i++) {
    target_trajectories_msg.input_trajectory[i].value = std::vector<float>(
      input_trajectory[i].data(), input_trajectory[i].data() + input_trajectory[i].size());
  }

  return target_trajectories_msg;
}

TargetTrajectories read_target_trajectories_msg(
  const MpcTargetTrajectoriesMsg & target_trajectories_msg)
{
  size_t N = target_trajectories_msg.state_trajectory.size();
  if (N == 0) {
    throw std::runtime_error("An empty target trajectories message is received.");
  }

  TargetTrajectories traj(N);
  scalar_array_t desiredTimeTrajectory(N);
  vector_array_t desiredStateTrajectory(N);
  for (size_t i = 0; i < N; i++) {
    traj.timeTrajectory[i] = target_trajectories_msg.time_trajectory[i];
    traj.stateTrajectory[i] = Eigen::Map<const Eigen::VectorXf>(
                                target_trajectories_msg.state_trajectory[i].value.data(),
                                target_trajectories_msg.state_trajectory[i].value.size())
                                .cast<scalar_t>();
  }
  for (size_t i = 0; i < N; i++) {
    traj.inputTrajectory[i] = Eigen::Map<const Eigen::VectorXf>(
                                target_trajectories_msg.input_trajectory[i].value.data(),
                                target_trajectories_msg.input_trajectory[i].value.size())
                                .cast<scalar_t>();
  }

  return traj;
}

ConstraintMsg create_constraint_msg(scalar_t time, const vector_t & constraint)
{
  ConstraintMsg constraint_msg;

  constraint_msg.time = time;
  constraint_msg.value.resize(constraint.size());
  for (size_t i = 0; i < constraint.size(); i++) {
    constraint_msg.value[i] = constraint(i);
  }

  return constraint_msg;
}

LagrangianMetricsMsg create_lagrangian_metrics_msg(scalar_t time, LagrangianMetricsConstRef metrics)
{
  LagrangianMetricsMsg metrics_msg;

  metrics_msg.time = time;
  metrics_msg.penalty = metrics.penalty;

  metrics_msg.constraint.resize(metrics.constraint.size());
  for (size_t i = 0; i < metrics.constraint.size(); i++) {
    metrics_msg.constraint[i] = metrics.constraint(i);
  }

  return metrics_msg;
}

MultiplierMsg create_multiplier_msg(scalar_t time, MultiplierConstRef multiplier)
{
  MultiplierMsg multiplier_msg;

  multiplier_msg.time = time;
  multiplier_msg.penalty = multiplier.penalty;

  multiplier_msg.lagrangian.resize(multiplier.lagrangian.size());
  for (size_t i = 0; i < multiplier.lagrangian.size(); i++) {
    multiplier_msg.lagrangian[i] = multiplier.lagrangian(i);
  }

  return multiplier_msg;
}

}  // namespace ros_msg_conversions
}  // namespace ocs2
