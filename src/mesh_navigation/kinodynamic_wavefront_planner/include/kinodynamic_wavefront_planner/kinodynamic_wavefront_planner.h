#ifndef MESH_NAVIGATION__KINODYNAMIC_WAVEFRONT_PLANNER_H
#define MESH_NAVIGATION__KINODYNAMIC_WAVEFRONT_PLANNER_H

#include "wave_front_planner/wave_front_planner.h"
#include <kinodynamic_wavefront_planner/KinodynamicWavefrontPlannerConfig.h>

namespace kinodynamic_wavefront_planner
{
class KinodynamicWavefrontPlanner : public wave_front_planner::WaveFrontPlanner
{
public:
  typedef boost::shared_ptr<KinodynamicWavefrontPlanner> Ptr;

  /**
   * @brief Constructor
   */
  KinodynamicWavefrontPlanner();

  /**
   * @brief Destructor
   */
  virtual ~KinodynamicWavefrontPlanner();


  virtual uint32_t makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost, std::string& message) override;
  virtual float getKinodynamicCost(const lvr2::VertexHandle& from, const lvr2::VertexHandle& to);
  virtual std::vector<lvr2::VertexHandle> getAdjacentVertices(const lvr2::VertexHandle& vertex);
  virtual std::vector<lvr2::VertexHandle> findMinimalCostPath(const mesh_map::Vector& original_start, const mesh_map::Vector& original_goal, std::function<double(const lvr2::VertexHandle&, const lvr2::VertexHandle&)> cost_function);
  virtual nav_msgs::Path getPathFromPoints(const std::vector<lvr2::VertexHandle> &path);
  virtual double calculateSteeringAngle(const std::vector<double>& current_state, const std::vector<double>& next_state, double L, double dt);
  virtual float getSteeringAngleCost(const lvr2::VertexHandle& from, const lvr2::VertexHandle& to);
  virtual nav_msgs::Path getCvpPath(std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>>& path, const mesh_map::Vector& goal_vec, double& cost);
  /**
   * @brief Dynamic reconfigure callback specific to KinodynamicWavefrontPlanner
   */
  void reconfigureCallback(kinodynamic_wavefront_planner::KinodynamicWavefrontPlannerConfig& cfg, uint32_t level);

private:
  KinodynamicWavefrontPlannerConfig kinodynamic_config;

};

}  // namespace kinodynamic_wavefront_planner

#endif  // MESH_NAVIGATION__KINODYNAMIC_WAVEFRONT_PLANNER_H
