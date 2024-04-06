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

#pragma once

#include <chrono>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "ocs2_ros_interfaces/common/ros_msg_conversions.hpp"
#include "ocs2_ros_interfaces/visualization/visualization_colors.hpp"

// ROS messages
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace ocs2
{

std_msgs::msg::ColorRGBA get_color(Color color, double alpha = 1.0);

void set_visible(MarkerMsg & marker);

void set_invisible(MarkerMsg & marker);

std_msgs::msg::Header get_header_msg(
  const std::string & frame_id, const builtin_interfaces::msg::Time & timeStamp);

template <typename It>
void assign_header(It firstIt, It lastIt, const std_msgs::msg::Header & header)
{
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->header = header;
  }
}

template <typename It>
void assign_increasing_id(It firstIt, It lastIt, int startId = 0)
{
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->id = startId++;
  }
}

MarkerMsg get_line_msg(
  std::vector<geometry_msgs::msg::Point> && points, Color color, double lineWidth);

geometry_msgs::msg::Point get_point_msg(const Eigen::Vector3d & point);

geometry_msgs::msg::Vector3 get_vector_msg(const Eigen::Vector3d & vec);

geometry_msgs::msg::Quaternion get_orientation_msg(const Eigen::Quaterniond & orientation);

MarkerMsg get_sphere_msg(const Eigen::Vector3d & point, Color color, double diameter);

MarkerMsg get_plane_msg(
  const Eigen::Vector3d & point, const Eigen::Quaterniond & orientation, Color color, double width,
  double length, double thickness);

MarkerMsg get_arrow_to_point_msg(
  const Eigen::Vector3d & vec, const Eigen::Vector3d & point, Color color);

MarkerMsg get_arrow_at_point_msg(
  const Eigen::Vector3d & vec, const Eigen::Vector3d & point, Color color);

MarkerMsg get_arrow_between_points_msg(
  const Eigen::Vector3d & start, const Eigen::Vector3d & end, Color color);

MarkerMsg get_foot_marker(
  const Eigen::Vector3d & position, bool contactFlag, Color color, double diameter,
  double liftedAlpha);

MarkerMsg get_force_marker(
  const Eigen::Vector3d & force, const Eigen::Vector3d & position, bool contactFlag, Color color,
  double forceScale);

template <typename ForceIt, typename PositionIt, typename ContactIt>
MarkerMsg get_center_of_pressure_marker(
  ForceIt firstForce, ForceIt lastForce, PositionIt positionIt, ContactIt contactIt, Color color,
  double diameter)
{
  // Compute center of pressure
  Eigen::Vector3d centerOfPressure = Eigen::Vector3d::Zero();
  double sum_z = 0.0;
  int numContacts = 0;
  for (; firstForce != lastForce; ++firstForce, ++positionIt, ++contactIt) {
    sum_z += firstForce->z();
    centerOfPressure += firstForce->z() * (*positionIt);
    numContacts += (*contactIt) ? 1 : 0;
  }
  if (sum_z > 0) {
    centerOfPressure /= sum_z;
  }

  // Construct marker
  MarkerMsg copMarker = get_sphere_msg(centerOfPressure, color, diameter);
  if (numContacts == 0) {
    set_invisible(copMarker);
  }
  copMarker.ns = "Center of Pressure";
  copMarker.pose.orientation = get_orientation_msg({1., 0., 0., 0.});
  return copMarker;
}

template <typename PositionIt, typename ContactIt>
MarkerMsg get_support_polygon_marker(
  PositionIt firstPos, PositionIt lastPos, ContactIt contactIt, Color color, double lineWidth)
{
  MarkerMsg lineList;
  lineList.type = MarkerMsg::LINE_LIST;
  auto numElements = std::distance(firstPos, lastPos);
  lineList.points.reserve(
    numElements * (numElements - 1) / 2);  // Upper bound on the number of lines

  // Loop over all positions
  for (; firstPos != lastPos; ++firstPos, ++contactIt) {
    // For each position, loop over all future positions in the container
    auto nextPos = std::next(firstPos);
    auto nextContact = std::next(contactIt);
    for (; nextPos != lastPos; ++nextPos, ++nextContact) {
      if (*contactIt && *nextContact) {
        // When positions are both marked as in contact, draw a line between the two points
        lineList.points.push_back(get_point_msg(*firstPos));
        lineList.points.push_back(get_point_msg(*nextPos));
      }
    }
  }
  lineList.scale.x = lineWidth;
  lineList.color = get_color(color);
  lineList.ns = "Support Polygon";
  lineList.pose.orientation = get_orientation_msg({1., 0., 0., 0.});
  return lineList;
}

template <typename F>
double timed_execution_in_seconds(F func)
{
  auto start = std::chrono::steady_clock::now();
  func();
  auto finish = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::duration<double>>(finish - start).count();
}

}  // namespace ocs2
