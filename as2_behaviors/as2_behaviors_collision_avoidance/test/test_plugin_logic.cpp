#include <gtest/gtest.h>
#include "as2_behaviors_collision_avoidance/collision_avoidance_base.hpp"
#include "as2_behaviors_collision_avoidance/path_geometry.hpp"

// Helper to build a path with PoseStampedWithID elements
static std::vector<as2_msgs::msg::PoseStampedWithID> make_path(
  double x0, double y0, double x1, double y1, const std::string & id_prefix = "p")
{
  std::vector<as2_msgs::msg::PoseStampedWithID> path;
  as2_msgs::msg::PoseStampedWithID p0, p1;
  p0.id = id_prefix + "0";
  p0.pose.pose.position.x = x0;
  p0.pose.pose.position.y = y0;
  p1.id = id_prefix + "1";
  p1.pose.pose.position.x = x1;
  p1.pose.pose.position.y = y1;
  path.push_back(p0);
  path.push_back(p1);
  return path;
}

// Basic placeholder test to verify the plugin can be linked
TEST(PluginLogic, BasicCompile)
{
  // This test verifies that the plugin headers compile correctly
  // and basic path geometry functions work.
  auto path_a = make_path(0, 0, 1, 0);
  auto path_b = make_path(0, 2, 1, 2);
  double dist = path_geometry::min_polyline_distance(path_a, path_b);
  EXPECT_NEAR(dist, 2.0, 1e-9);
}

// Test that paths don't conflict when far apart
TEST(PluginLogic, NonConflictingPaths)
{
  auto path_a = make_path(0, 0, 1, 0, "a");
  auto path_b = make_path(0, 5, 1, 5, "b");
  double dist = path_geometry::min_polyline_distance(path_a, path_b);
  // Minimum distance should be 5 (y-axis separation)
  EXPECT_NEAR(dist, 5.0, 1e-9);
}

// Test that crossing paths have zero distance
TEST(PluginLogic, CrossingPaths)
{
  auto path_a = make_path(-1, 0, 1, 0, "a");
  auto path_b = make_path(0, -1, 0, 1, "b");
  double dist = path_geometry::min_polyline_distance(path_a, path_b);
  EXPECT_NEAR(dist, 0.0, 1e-9);
}
