#include <gtest/gtest.h>
#include "as2_behaviors_collision_avoidance/path_geometry.hpp"

using as2_msgs::msg::PoseStampedWithID;
using geometry_msgs::msg::PoseStamped;

static PoseStampedWithID make_pose(double x, double y, double z = 0.0, const std::string & id = "")
{
  PoseStampedWithID ps;
  ps.id = id;
  ps.pose.pose.position.x = x;
  ps.pose.pose.position.y = y;
  ps.pose.pose.position.z = z;
  return ps;
}

static std::vector<PoseStampedWithID> make_path(std::initializer_list<PoseStampedWithID> poses)
{
  return std::vector<PoseStampedWithID>(poses);
}

TEST(PathGeometry, ParallelSegments)
{
  auto a = make_path({make_pose(0, 0, 0, "a0"), make_pose(1, 0, 0, "a1")});
  auto b = make_path({make_pose(0, 2, 0, "b0"), make_pose(1, 2, 0, "b1")});
  EXPECT_NEAR(path_geometry::min_polyline_distance(a, b), 2.0, 1e-9);
}

TEST(PathGeometry, CrossingSegments)
{
  auto a = make_path({make_pose(-1, 0, 0, "a0"), make_pose(1, 0, 0, "a1")});
  auto b = make_path({make_pose(0, -1, 0, "b0"), make_pose(0, 1, 0, "b1")});
  EXPECT_NEAR(path_geometry::min_polyline_distance(a, b), 0.0, 1e-9);
}

TEST(PathGeometry, Disjoint)
{
  auto a = make_path({make_pose(0, 0, 0, "a0"), make_pose(1, 0, 0, "a1")});
  auto b = make_path({make_pose(3, 0, 0, "b0"), make_pose(4, 0, 0, "b1")});
  EXPECT_NEAR(path_geometry::min_polyline_distance(a, b), 2.0, 1e-9);
}

TEST(PathGeometry, PointVsSegment)
{
  auto a = make_path({make_pose(0, 0, 0, "a0")});
  auto b = make_path({make_pose(0, 3, 0, "b0"), make_pose(5, 3, 0, "b1")});
  EXPECT_NEAR(path_geometry::min_polyline_distance(a, b), 3.0, 1e-9);
}

TEST(PathGeometry, SkewLines3D)
{
  // x-axis vs y-axis shifted 1 unit in z — min dist = 1.
  auto a = make_path({make_pose(-1, 0, 0, "a0"), make_pose(1, 0, 0, "a1")});
  auto b = make_path({make_pose(0, -1, 1, "b0"), make_pose(0, 1, 1, "b1")});
  EXPECT_NEAR(path_geometry::min_polyline_distance(a, b), 1.0, 1e-9);
}
