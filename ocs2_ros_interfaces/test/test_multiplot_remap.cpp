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

#include <string>

#include "ocs2_core/Types.hpp"
#include "ocs2_ros_interfaces/common/ros_msg_conversions.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class MultiplotRemap : public rclcpp::Node
{
public:
  explicit MultiplotRemap(const std::string & mpc_policy_topic_name) : Node("multiplot_remap")
  {
    mpc_policy_sub_ = this->create_subscription<ocs2::MpcFlattenedControllerMsg>(
      mpc_policy_topic_name, rclcpp::ParametersQoS(),
      std::bind(&MultiplotRemap::mpc_poicy_cb, this, std::placeholders::_1));
    mpc_performance_indices_pub_ = this->create_publisher<ocs2::MpcPerformanceIndicesMsg>(
      "mpc_performance_indices", rclcpp::ParametersQoS());
  }

  /** Default deconstructor */
  ~MultiplotRemap() = default;

private:
  void mpc_poicy_cb(const ocs2::MpcFlattenedControllerMsg::ConstSharedPtr & policy_msg)
  {
    mpc_performance_indices_pub_->publish(policy_msg->performance_indices);
  }

  rclcpp::Subscription<ocs2::MpcFlattenedControllerMsg>::SharedPtr mpc_policy_sub_;
  rclcpp::Publisher<ocs2::MpcPerformanceIndicesMsg>::SharedPtr mpc_performance_indices_pub_;
};

}  // unnamed namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto arguments = rclcpp::remove_ros_arguments(argc, argv);
  if (arguments.size() <= 1) {
    throw std::runtime_error("MPC policy topic name is not specified!");
  }

  const std::string topic_name = std::string(arguments[1]);
  auto node = MultiplotRemap(topic_name);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node.get_node_base_interface());
  exec->spin();
  rclcpp::shutdown();

  return 0;
}
