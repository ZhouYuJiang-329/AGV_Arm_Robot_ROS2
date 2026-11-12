#include "agv_bringup/measured_ray_obstacle_layer.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace agv_bringup
{
void MeasuredRayObstacleLayer::onInitialize()
{
  nav2_costmap_2d::ObstacleLayer::onInitialize();
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("costmap node expired");}
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  const auto parameter = name_ + ".clearing_max_age_sec";
  if (!node->has_parameter(parameter)) {
    node->declare_parameter(parameter, 0.6, descriptor);
  }
  clearing_max_age_ = node->get_parameter(parameter).as_double();
  if (!std::isfinite(clearing_max_age_) || clearing_max_age_ <= 0) {
    throw std::invalid_argument("clearing_max_age_sec must be positive and finite");
  }
}

void MeasuredRayObstacleLayer::raytraceFreespace(
  const nav2_costmap_2d::Observation & observation,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  auto node = node_.lock();
  if (!node || !observation.cloud_ || observation.cloud_->data.empty() ||
    observation.cloud_->header.frame_id != global_frame_) {return;}
  const auto stamp = rclcpp::Time(observation.cloud_->header.stamp);
  const double age = (node->now() - stamp).seconds();
  if (stamp.nanoseconds() <= 0 || age < 0 || age >= clearing_max_age_) {return;}
  const double ox = observation.origin_.x, oy = observation.origin_.y;
  const double maximum = observation.raytrace_max_range_;
  const double minimum = observation.raytrace_min_range_;
  unsigned int origin_cell_x, origin_cell_y;
  if (!std::isfinite(ox) || !std::isfinite(oy) || !std::isfinite(maximum) ||
    !std::isfinite(minimum) || minimum < 0 || maximum <= minimum ||
    !worldToMap(ox, oy, origin_cell_x, origin_cell_y)) {return;}
  const double map_max_x = origin_x_ + size_x_ * resolution_;
  const double map_max_y = origin_y_ + size_y_ * resolution_;
  const double infinity = std::numeric_limits<double>::infinity();
  sensor_msgs::PointCloud2ConstIterator<float> x(*observation.cloud_, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(*observation.cloud_, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(*observation.cloud_, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
    const double dx = *x - ox, dy = *y - oy;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(distance) || distance <= minimum) {continue;}
    const double ux = dx / distance, uy = dy / distance;
    double end = std::min(distance, maximum);
    // Clip to the grid without moving either measured endpoint or sensor origin.
    if (ux > 0) {end = std::min(end, (map_max_x - ox) / ux);}
    if (ux < 0) {end = std::min(end, (origin_x_ - ox) / ux);}
    if (uy > 0) {end = std::min(end, (map_max_y - oy) / uy);}
    if (uy < 0) {end = std::min(end, (origin_y_ - oy) / uy);}
    if (end <= minimum) {continue;}
    unsigned int hit_x, hit_y;
    const bool hit_in_map = worldToMap(*x, *y, hit_x, hit_y);
    double sx = ox + ux * minimum, sy = oy + uy * minimum;
    if (ux != 0) {sx = std::nextafter(sx, ox + ux * end);}
    if (uy != 0) {sy = std::nextafter(sy, oy + uy * end);}
    unsigned int start_x, start_y;
    if (!worldToMap(sx, sy, start_x, start_y)) {continue;}
    int ix = static_cast<int>(start_x), iy = static_cast<int>(start_y);
    const int step_x = ux > 0 ? 1 : ux < 0 ? -1 : 0;
    const int step_y = uy > 0 ? 1 : uy < 0 ? -1 : 0;
    double next_x = step_x == 0 ? infinity :
      (origin_x_ + (ix + (step_x > 0 ? 1 : 0)) * resolution_ - ox) / ux;
    double next_y = step_y == 0 ? infinity :
      (origin_y_ + (iy + (step_y > 0 ? 1 : 0)) * resolution_ - oy) / uy;
    const double delta_x = step_x == 0 ? infinity : resolution_ / std::abs(ux);
    const double delta_y = step_y == 0 ? infinity : resolution_ / std::abs(uy);
    int low_x = ix, high_x = ix, low_y = iy, high_y = iy;
    bool cleared = false;
    for (unsigned int step = 0; step <= size_x_ + size_y_; ++step) {
      if (ix < 0 || iy < 0 || ix >= static_cast<int>(size_x_) ||
        iy >= static_cast<int>(size_y_)) {break;}
      // Never treat the measured return cell (or anything behind it) as free.
      // This also protects hits outside the configured obstacle marking range.
      if (hit_in_map && ix == static_cast<int>(hit_x) && iy == static_cast<int>(hit_y)) {
        break;
      }
      costmap_[getIndex(ix, iy)] = nav2_costmap_2d::FREE_SPACE;
      low_x = std::min(low_x, ix); high_x = std::max(high_x, ix);
      low_y = std::min(low_y, iy); high_y = std::max(high_y, iy); cleared = true;
      const double boundary_x = next_x, boundary_y = next_y;
      if (std::min(boundary_x, boundary_y) >= end) {break;}
      // At an exact corner, enter the diagonal cell; don't clear side cells
      // touched only at a zero-area corner.
      if (boundary_x <= boundary_y) {ix += step_x; next_x += delta_x;}
      if (boundary_y <= boundary_x) {iy += step_y; next_y += delta_y;}
    }
    if (cleared) {
      *min_x = std::min(*min_x, origin_x_ + low_x * resolution_);
      *min_y = std::min(*min_y, origin_y_ + low_y * resolution_);
      *max_x = std::max(*max_x, origin_x_ + (high_x + 1) * resolution_);
      *max_y = std::max(*max_y, origin_y_ + (high_y + 1) * resolution_);
    }
  }
}
}
PLUGINLIB_EXPORT_CLASS(agv_bringup::MeasuredRayObstacleLayer, nav2_costmap_2d::Layer)
