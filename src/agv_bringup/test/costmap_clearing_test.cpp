#include "agv_bringup/measured_ray_obstacle_layer.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <iostream>
#include <limits>
#include <vector>

using ClearingLayer = agv_bringup::MeasuredRayObstacleLayer;
namespace
{
void require(bool value, const char * message)
{if (!value) {throw std::runtime_error(message);}}
nav2_costmap_2d::Observation observation(
  const rclcpp::Time & stamp, double range, bool empty = false)
{
  // Translated live scan/map fixture: 2cm grid, sub-cell sensor origin.
  geometry_msgs::msg::Point origin;
  origin.x = 1.778636671615047; origin.y = 2.0015091581232965; origin.z = -0.24;
  constexpr double angle = 0.47907285790908527;
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map"; cloud.header.stamp = stamp;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz"); modifier.resize(empty ? 0 : 1);
  if (!empty) {
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    *x = origin.x + range * std::cos(angle); *y = origin.y + range * std::sin(angle);
    *z = origin.z;
  }
  return nav2_costmap_2d::Observation(origin, cloud, 3.5, 0.0, 4.0, 0.0);
}
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<nav2_util::LifecycleNode>("clearing_regression");
    node->declare_parameter("track_unknown_space", false);
    node->declare_parameter("transform_tolerance", 0.3);
    node->declare_parameter("obstacle_layer.observation_sources", std::string(""));
    node->declare_parameter("obstacle_layer.footprint_clearing_enabled", false);
    node->declare_parameter("obstacle_layer.min_obstacle_height", -1.0);
    node->declare_parameter("obstacle_layer.max_obstacle_height", 1.0);
    tf2_ros::Buffer buffer(node->get_clock());
    nav2_costmap_2d::LayeredCostmap map("map", false, false);
    constexpr double resolution = 0.019999999552965164;
    map.resizeMap(400, 300, resolution, 0, 0);
    auto layer = std::make_shared<ClearingLayer>();
    layer->initialize(&map, "obstacle_layer", &buffer, node,
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    auto update = [&] {
        double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
        layer->updateBounds(2, 2, 0, &min_x, &min_y, &max_x, &max_y);
      };
    auto person = observation(node->now(), 2.1429998874664307);
    layer->addStaticObservation(person, true, true); update();
    constexpr unsigned int target_x = 184, target_y = 149;
    require(layer->getCost(target_x, target_y) == nav2_costmap_2d::LETHAL_OBSTACLE,
      "fixture failed to mark the former pedestrian cell");
    layer->clearStaticObservations(true, true);
    auto unknown = observation(node->now(), 0, true);
    layer->addStaticObservation(unknown, false, true); update();
    require(layer->getCost(target_x, target_y) == nav2_costmap_2d::LETHAL_OBSTACLE,
      "missing/unknown observations must not clear obstacles");
    layer->clearStaticObservations(true, true);
    auto invalid = observation(node->now(), std::numeric_limits<double>::quiet_NaN());
    layer->addStaticObservation(invalid, false, true); update();
    require(layer->getCost(target_x, target_y) == nav2_costmap_2d::LETHAL_OBSTACLE,
      "NaN returns must not clear obstacles");
    layer->clearStaticObservations(true, true);
    auto infinite = observation(node->now(), std::numeric_limits<double>::infinity());
    layer->addStaticObservation(infinite, false, true); update();
    require(layer->getCost(target_x, target_y) == nav2_costmap_2d::LETHAL_OBSTACLE,
      "infinite returns must not clear obstacles");
    layer->clearStaticObservations(true, true);
    auto expired = observation(node->now() - rclcpp::Duration::from_seconds(1), 5.215000152587891);
    layer->addStaticObservation(expired, false, true); update();
    require(layer->getCost(target_x, target_y) == nav2_costmap_2d::LETHAL_OBSTACLE,
      "stale measurements must not clear obstacles");
    layer->clearStaticObservations(true, true);
    auto future = observation(node->now() + rclcpp::Duration::from_seconds(1), 5.215000152587891);
    layer->addStaticObservation(future, false, true); update();
    require(layer->getCost(target_x, target_y) == nav2_costmap_2d::LETHAL_OBSTACLE,
      "future-dated measurements must not clear obstacles");
    layer->clearStaticObservations(true, true);
    // A real return and its shadow must remain occupied even in clearing-only input.
    layer->addStaticObservation(person, false, true); update();
    require(layer->getCost(target_x, target_y) == nav2_costmap_2d::LETHAL_OBSTACLE,
      "clearing erased a real return cell");
    layer->clearStaticObservations(true, true);
    layer->setCost(target_x, target_y - 1, nav2_costmap_2d::LETHAL_OBSTACLE);
    constexpr double origin_x = 1.778636671615047, origin_y = 2.0015091581232965;
    constexpr double angle = 0.47907285790908527;
    unsigned int distant_x, distant_y;
    require(layer->worldToMap(origin_x + 4.5 * std::cos(angle), origin_y + 4.5 * std::sin(angle),
      distant_x, distant_y), "range fixture outside grid");
    layer->setCost(distant_x, distant_y, nav2_costmap_2d::LETHAL_OBSTACLE);
    auto wall = observation(node->now(), 5.215000152587891);
    layer->addStaticObservation(wall, false, true); update();
    std::cout << "former pedestrian cell cost=" << static_cast<int>(
      layer->getCost(target_x, target_y)) << '\n';
    require(layer->getCost(target_x, target_y) == nav2_costmap_2d::FREE_SPACE,
      "finite measured free ray did not clear former obstacle");
    require(layer->getCost(target_x, target_y - 1) == nav2_costmap_2d::LETHAL_OBSTACLE,
      "an unobserved neighbouring cell was erased");
    require(layer->getCost(distant_x, distant_y) == nav2_costmap_2d::LETHAL_OBSTACLE,
      "raytrace range cap was ignored");
    // The same world traversal must work in the negative direction and preserve its hit.
    layer->clearStaticObservations(true, true);
    auto reverse = observation(node->now(), 5.215000152587891);
    const auto point = *reverse.cloud_;
    sensor_msgs::PointCloud2ConstIterator<float> px(point, "x"), py(point, "y");
    reverse.origin_.x = *px; reverse.origin_.y = *py;
    sensor_msgs::PointCloud2Iterator<float> ex(*reverse.cloud_, "x"), ey(*reverse.cloud_, "y");
    *ex = origin_x; *ey = origin_y;
    layer->setCost(target_x, target_y, nav2_costmap_2d::LETHAL_OBSTACLE);
    layer->addStaticObservation(reverse, false, true); update();
    require(layer->getCost(target_x, target_y) == nav2_costmap_2d::FREE_SPACE,
      "negative-direction measured ray failed to clear");
    // Axis-aligned, diagonal and map-boundary rays exercise DDA edge cases.
    for (const auto & endpoint : std::vector<std::pair<double, double>>{
      {4.005, 1.005}, {1.005, 4.005}, {4.005, 4.005}, {-1.0, 1.005}})
    {
      layer->clearStaticObservations(true, true);
      layer->resetMap(0, 0, layer->getSizeInCellsX(), layer->getSizeInCellsY());
      auto ray = observation(node->now(), 1);
      ray.origin_.x = 1.005; ray.origin_.y = 1.005;
      sensor_msgs::PointCloud2Iterator<float> rx(*ray.cloud_, "x"), ry(*ray.cloud_, "y");
      *rx = endpoint.first; *ry = endpoint.second;
      const bool vertical = endpoint.first == 1.005;
      const bool diagonal = endpoint.first == endpoint.second;
      const bool outside = endpoint.first < 0;
      const unsigned int test_x = vertical ? 50 : outside ? 25 : 100;
      const unsigned int test_y = vertical || diagonal ? 100 : 50;
      layer->setCost(test_x, test_y, nav2_costmap_2d::LETHAL_OBSTACLE);
      layer->setCost(100, 99, nav2_costmap_2d::LETHAL_OBSTACLE);
      layer->addStaticObservation(ray, false, true); update();
      require(layer->getCost(test_x, test_y) == nav2_costmap_2d::FREE_SPACE,
        "axis/diagonal/boundary measured ray failed");
      if (diagonal) {
        require(layer->getCost(100, 99) == nav2_costmap_2d::LETHAL_OBSTACLE,
          "zero-area corner contact cleared an unobserved side cell");
      }
    }

  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n'; rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
