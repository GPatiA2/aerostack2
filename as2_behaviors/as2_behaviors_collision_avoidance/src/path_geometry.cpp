// Copyright 2026 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!*******************************************************************************************
 *  \file       path_geometry.cpp
 *  \brief      Path geometry utilities for collision avoidance
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include "as2_behaviors_collision_avoidance/path_geometry.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <limits>

namespace path_geometry
{

namespace
{

Eigen::Vector3d to_eigen(const geometry_msgs::msg::PoseStamped & ps)
{
  return {ps.pose.position.x, ps.pose.position.y, ps.pose.position.z};
}

/// Closest distance between two finite 3-D line segments [p, p+d] and [q, q+e].
double segment_to_segment(
  Eigen::Vector3d p, Eigen::Vector3d d,
  Eigen::Vector3d q, Eigen::Vector3d e)
{
  // Parameterise as P(s) = p + s*d, Q(t) = q + t*e, s,t ∈ [0,1].
  double dd = d.dot(d);
  double ee = e.dot(e);
  Eigen::Vector3d r = p - q;

  // Degenerate cases: one or both segments are points.
  if (dd < 1e-12 && ee < 1e-12) {
    return r.norm();
  }
  if (dd < 1e-12) {
    double t = std::clamp(e.dot(r) / ee, 0.0, 1.0);  // closest t to point p
    t = std::clamp(-r.dot(e) / ee, 0.0, 1.0);
    return (p - (q + t * e)).norm();
  }
  if (ee < 1e-12) {
    double s = std::clamp(-r.dot(d) / dd, 0.0, 1.0);
    return (p + s * d - q).norm();
  }

  double de = d.dot(e);
  double denom = dd * ee - de * de;  // 0 iff parallel

  double s, t;
  if (std::abs(denom) > 1e-12) {
    s = std::clamp((de * e.dot(r) - ee * d.dot(r)) / denom, 0.0, 1.0);
  } else {
    s = 0.0;  // parallel: pick one end
  }
  t = (e.dot(r) + s * de) / ee;

  if (t < 0.0) {
    t = 0.0;
    s = std::clamp(-d.dot(r) / dd, 0.0, 1.0);
  } else if (t > 1.0) {
    t = 1.0;
    s = std::clamp((de - d.dot(r)) / dd, 0.0, 1.0);
  }

  return (p + s * d - q - t * e).norm();
}

}  // namespace

double min_polyline_distance(
  const std::vector<as2_msgs::msg::PoseStampedWithID> & a,
  const std::vector<as2_msgs::msg::PoseStampedWithID> & b)
{
  if (a.empty() || b.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double min_d = std::numeric_limits<double>::infinity();

  // If a path has only one point, treat it as a zero-length segment.
  size_t na = a.size();
  size_t nb = b.size();

  for (size_t i = 0; i < (na > 1 ? na - 1 : 1); ++i) {
    Eigen::Vector3d a0 = to_eigen(a[i].pose);
    Eigen::Vector3d da;
    if (na > 1) {
      da = to_eigen(a[i + 1].pose) - a0;
    } else {
      da = Eigen::Vector3d::Zero();
    }
    for (size_t j = 0; j < (nb > 1 ? nb - 1 : 1); ++j) {
      Eigen::Vector3d b0 = to_eigen(b[j].pose);
      Eigen::Vector3d db;
      if (nb > 1) {
        db = to_eigen(b[j + 1].pose) - b0;
      } else {
        db = Eigen::Vector3d::Zero();
      }
      min_d = std::min(min_d, segment_to_segment(a0, da, b0, db));
    }
  }
  return min_d;
}

}  // namespace path_geometry
