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

#include "ocs2_ros_interfaces/common/ros_msg_helpers.hpp"

namespace ocs2
{
namespace ros_msg_helpers
{

PointMsg get_point_msg(const Eigen::Vector3d & point)
{
  PointMsg point_msg;
  point_msg.x = point.x();
  point_msg.y = point.y();
  point_msg.z = point.z();
  return point_msg;
}

Vector3Msg get_vector_msg(const Eigen::Vector3d & vec)
{
  Vector3Msg vec_msg;
  vec_msg.x = vec.x();
  vec_msg.y = vec.y();
  vec_msg.z = vec.z();
  return vec_msg;
}

QuaternionMsg get_orientation_msg(const Eigen::Quaterniond & orientation)
{
  QuaternionMsg orientation_msg;
  orientation_msg.x = orientation.x();
  orientation_msg.y = orientation.y();
  orientation_msg.z = orientation.z();
  orientation_msg.w = orientation.w();
  return orientation_msg;
}

HeaderMsg get_header_msg(
  const std::string & frame_id, const builtin_interfaces::msg::Time & time_stamp)
{
  HeaderMsg header;
  header.frame_id = frame_id;
  header.stamp = time_stamp;
  return header;
}

MarkerMsg get_line_msg(
  std::vector<PointMsg> && points, std::array<double, 3> color, double lineWidth)
{
  MarkerMsg line;
  line.type = MarkerMsg::LINE_STRIP;
  line.scale.x = lineWidth;
  line.color = get_color(color);
  line.points = std::move(points);
  line.pose.orientation = get_orientation_msg({1., 0., 0., 0.});
  return line;
}

ColorRGBAMsg get_color(std::array<double, 3> rgb, double alpha)
{
  ColorRGBAMsg color_msg;
  color_msg.r = rgb[0];
  color_msg.g = rgb[1];
  color_msg.b = rgb[2];
  color_msg.a = alpha;
  return color_msg;
}

}  // namespace ros_msg_helpers
}  // namespace ocs2
