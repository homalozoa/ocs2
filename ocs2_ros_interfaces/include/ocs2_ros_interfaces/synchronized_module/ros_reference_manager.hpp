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

#include <memory>
#include <string>
#include <utility>

#include "ocs2_oc/synchronized_module/ReferenceManagerDecorator.hpp"
#include "ocs2_ros_interfaces/common/ros_msg_conversions.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ocs2
{

/**
 * Decorates ReferenceManager with ROS subscribers to receive ModeSchedule and TargetTrajectories through ROS messages.
 */
class RosReferenceManager final : public ReferenceManagerDecorator
{
public:
  template <class NodePtrT>
  explicit RosReferenceManager(
    NodePtrT && nodeptr, std::string topic_prefix,
    std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
    const rclcpp::QoS qos = rclcpp::ParametersQoS());
  ~RosReferenceManager() noexcept;
  /**
   * Creates a pointer to RosReferenceManager using a the derived class of type ReferenceManagerInterface, i.e.
   * DerivedReferenceManager(args...).
   *
   * @param [in] topicPrefix: The ReferenceManager will subscribe to "topicPrefix_mode_schedule" and "topicPrefix_mpc_target"
   * topics to receive user-commanded ModeSchedule and TargetTrajectories respectively.
   * @param args: arguments to forward to the constructor of DerivedReferenceManager
   */
  template <class ReferenceManagerType, class NodePtrT, class... Args>
  static std::unique_ptr<RosReferenceManager> create(
    NodePtrT && nodeptr, const std::string & topicPrefix, Args &&... args);

private:
  rclcpp::Subscription<ModeScheduleMsg>::SharedPtr mode_schedule_sub_;
  rclcpp::Subscription<MpcTargetTrajectoriesMsg>::SharedPtr target_trajs_sub_;
};

}  // namespace ocs2
