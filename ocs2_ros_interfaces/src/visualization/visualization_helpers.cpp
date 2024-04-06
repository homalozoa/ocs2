/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "ocs2_ros_interfaces/visualization/visualization_helpers.hpp"

namespace ocs2
{

std_msgs::msg::ColorRGBA get_color(Color color, double alpha)
{
  const auto rgb = get_rgb(color);
  std_msgs::msg::ColorRGBA colorMsg;
  colorMsg.r = rgb[0];
  colorMsg.g = rgb[1];
  colorMsg.b = rgb[2];
  colorMsg.a = alpha;
  return colorMsg;
}

void set_visible(MarkerMsg & marker) { marker.color.a = 1.0; }

void set_invisible(MarkerMsg & marker)
{
  marker.color.a = 0.001;  // Rviz creates a warning when a is set to 0
}

std_msgs::msg::Header get_header_msg(
  const std::string & frame_id, const builtin_interfaces::msg::Time & timeStamp)
{
  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp = timeStamp;
  return header;
}

MarkerMsg get_line_msg(
  std::vector<geometry_msgs::msg::Point> && points, Color color, double lineWidth)
{
  MarkerMsg line;
  line.type = MarkerMsg::LINE_STRIP;
  line.scale.x = lineWidth;
  line.color = get_color(color);
  line.points = std::move(points);
  line.pose.orientation = get_orientation_msg({1., 0., 0., 0.});
  return line;
}

geometry_msgs::msg::Point get_point_msg(const Eigen::Vector3d & point)
{
  geometry_msgs::msg::Point pointMsg;
  pointMsg.x = point.x();
  pointMsg.y = point.y();
  pointMsg.z = point.z();
  return pointMsg;
}

geometry_msgs::msg::Vector3 get_vector_msg(const Eigen::Vector3d & vec)
{
  geometry_msgs::msg::Vector3 vecMsg;
  vecMsg.x = vec.x();
  vecMsg.y = vec.y();
  vecMsg.z = vec.z();
  return vecMsg;
}

geometry_msgs::msg::Quaternion get_orientation_msg(const Eigen::Quaterniond & orientation)
{
  geometry_msgs::msg::Quaternion orientationMsg;
  orientationMsg.x = orientation.x();
  orientationMsg.y = orientation.y();
  orientationMsg.z = orientation.z();
  orientationMsg.w = orientation.w();
  return orientationMsg;
}

MarkerMsg get_sphere_msg(const Eigen::Vector3d & point, Color color, double diameter)
{
  MarkerMsg sphere;
  sphere.type = MarkerMsg::SPHERE;
  sphere.pose.position = get_point_msg(point);
  sphere.pose.orientation = get_orientation_msg({1., 0., 0., 0.});
  sphere.scale.x = diameter;
  sphere.scale.y = diameter;
  sphere.scale.z = diameter;
  sphere.color = get_color(color);
  return sphere;
}

MarkerMsg get_plane_msg(
  const Eigen::Vector3d & point, const Eigen::Quaterniond & orientation, Color color, double width,
  double length, double thickness)
{
  MarkerMsg plane;
  plane.type = MarkerMsg::CUBE;
  plane.pose.position = get_point_msg(point);
  plane.pose.orientation = get_orientation_msg(orientation);
  plane.scale.x = length;
  plane.scale.y = width;
  plane.scale.z = thickness;
  plane.color = get_color(color);
  return plane;
}

MarkerMsg get_arrow_to_point_msg(
  const Eigen::Vector3d & vec, const Eigen::Vector3d & point, Color color)
{
  return get_arrow_between_points_msg(point - vec, point, color);
}

MarkerMsg get_arrow_at_point_msg(
  const Eigen::Vector3d & vec, const Eigen::Vector3d & point, Color color)
{
  return get_arrow_between_points_msg(point, point + vec, color);
}

MarkerMsg get_arrow_between_points_msg(
  const Eigen::Vector3d & start, const Eigen::Vector3d & end, Color color)
{
  MarkerMsg arrow;
  arrow.type = MarkerMsg::ARROW;
  arrow.scale.x = 0.01;  // shaft diameter
  arrow.scale.y = 0.02;  // arrow-head diameter
  arrow.scale.z = 0.06;  // arrow-head length
  arrow.points.reserve(2);
  arrow.points.emplace_back(get_point_msg(start));  // start point
  arrow.points.emplace_back(get_point_msg(end));    // end point
  arrow.color = get_color(color);
  arrow.pose.orientation = get_orientation_msg({1., 0., 0., 0.});
  return arrow;
}

MarkerMsg getFootMarker(
  const Eigen::Vector3d & position, bool contactFlag, Color color, double diameter,
  double liftedAlpha)
{
  auto footMarker = get_sphere_msg(position, color, diameter);
  if (!contactFlag) {
    footMarker.color.a = liftedAlpha;
  }
  footMarker.ns = "EE Positions";
  return footMarker;
}

MarkerMsg get_force_marker(
  const Eigen::Vector3d & force, const Eigen::Vector3d & position, bool contactFlag, Color color,
  double forceScale)
{
  auto forceMarker = get_arrow_to_point_msg(force / forceScale, position, color);
  forceMarker.ns = "EE Forces";
  if (!contactFlag) {
    set_invisible(forceMarker);
  }
  return forceMarker;
}

}  // namespace ocs2
