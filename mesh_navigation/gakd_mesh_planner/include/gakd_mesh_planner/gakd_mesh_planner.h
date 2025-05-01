#ifndef MESH_NAVIGATION__MESH_PLANNER_H
#define MESH_NAVIGATION__MESH_PLANNER_H

#include <mbf_mesh_core/mesh_planner.h>
#include <mbf_msgs/action/get_path.hpp>
#include <mesh_map/mesh_map.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gakd_mesh_planner
{
class GAKDMeshPlanner : public mbf_mesh_core::MeshPlanner
{
public:
  typedef std::shared_ptr<gakd_mesh_planner::GAKDMeshPlanner> Ptr;

  GAKDMeshPlanner();

  virtual ~GAKDMeshPlanner();

  virtual uint32_t makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                            double tolerance, std::vector<geometry_msgs::msg::PoseStamped>& plan, double& cost,
                            std::string& message) override;

  virtual bool cancel() override;

  virtual bool initialize(const std::string& plugin_name, const std::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr,
                          const rclcpp::Node::SharedPtr& node) override;

private:
  mesh_map::MeshMap::Ptr mesh_map_;

  std::string name_;

  rclcpp::Node::SharedPtr node_;

  std::atomic_bool cancel_planning_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::string map_frame_;
};

}  // namespace gakd_mesh_planner

#endif  // MESH_NAVIGATION__MESH_PLANNER_H