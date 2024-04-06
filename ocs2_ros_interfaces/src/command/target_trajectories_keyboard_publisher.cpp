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

#include "ocs2_ros_interfaces/command/target_trajectories_keyboard_publisher.hpp"

#include "ocs2_core/misc/CommandLine.hpp"
#include "ocs2_core/misc/Display.hpp"
#include "ocs2_msgs/msg/mpc_observation.hpp"
#include "ocs2_ros_interfaces/common/ros_msg_conversions.hpp"

namespace ocs2
{
void TargetTrajectoriesKeyboardPublisher::publish_keyboard_command(const std::string & cmd_msg)
{
  std::cout << cmd_msg << ": ";
  const vector_t cmdline_input =
    get_command_line().cwiseMin(target_command_limits_).cwiseMax(-target_command_limits_);
  std::cout << "The following command is published: [" << to_delimited_str(cmdline_input)
            << "]\n\n";
  SystemObservation observation;
  {
    std::lock_guard<std::mutex> lock(latest_observation_mutex_);
    observation = latest_observation_;
  }
  const auto trajs = func_cmdline_to_target_trajs_(cmdline_input, observation);
  target_trajs_publisher_->publish_traj(trajs);
}

vector_t TargetTrajectoriesKeyboardPublisher::get_command_line()
{
  auto should_terminate = []() { return rclcpp::ok(); };
  const std::string line = get_cmdline_str(should_terminate);
  const std::vector<std::string> words = str_to_words(line);
  const size_t target_command_size = target_command_limits_.size();
  vector_t target_command = vector_t::Zero(target_command_size);
  for (size_t i = 0; i < std::min(words.size(), target_command_size); i++) {
    target_command(i) = static_cast<scalar_t>(stof(words[i]));
  }
  return target_command;
}

}  // namespace ocs2
