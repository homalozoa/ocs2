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

#include <array>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace ocs2
{

using Vector3Msg = geometry_msgs::msg::Vector3;
using PointMsg = geometry_msgs::msg::Point;
using QuaternionMsg = geometry_msgs::msg::Quaternion;
using HeaderMsg = std_msgs::msg::Header;
using MarkerMsg = visualization_msgs::msg::Marker;
using ColorRGBAMsg = std_msgs::msg::ColorRGBA;

namespace ros_msg_helpers
{

Vector3Msg get_vector_msg(const Eigen::Vector3d & vec);
PointMsg get_point_msg(const Eigen::Vector3d & point);
QuaternionMsg get_orientation_msg(const Eigen::Quaterniond & orientation);
HeaderMsg get_header_msg(
  const std::string & frame_id, const builtin_interfaces::msg::Time & time_stamp);
MarkerMsg get_line_msg(
  std::vector<geometry_msgs::msg::Point> && points, std::array<double, 3> color, double line_width);
ColorRGBAMsg get_color(std::array<double, 3> rgb, double alpha = 1.0);

}  // namespace ros_msg_helpers
}  // namespace ocs2
