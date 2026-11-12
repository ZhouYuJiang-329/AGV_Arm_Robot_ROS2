#pragma once
#include <nav2_costmap_2d/obstacle_layer.hpp>

namespace agv_bringup
{
// Retains Nav2 marking, observation buffers and layer combination. Clearing uses
// the measured world ray rather than a line between quantized endpoint cells.
class MeasuredRayObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void onInitialize() override;

protected:
  void raytraceFreespace(
    const nav2_costmap_2d::Observation & observation,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

private:
  double clearing_max_age_ = 0.6;
};
}
