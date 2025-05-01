#include <lvr2/geometry/Handles.hpp>
#include <lvr2/util/Meap.hpp>
#include <mesh_map/util.h>
#include <pluginlib/class_list_macros.hpp>
#include "gakd_mesh_planner/gakd_mesh_planner.h"

PLUGINLIB_EXPORT_CLASS(gakd_mesh_planner::GAKDMeshPlanner, mbf_mesh_core::MeshPlanner);

namespace gakd_mesh_planner
{
GAKDMeshPlanner::GAKDMeshPlanner() : cancel_planning_(false)
{
}

GAKDMeshPlanner::~GAKDMeshPlanner()
{
}

bool GAKDMeshPlanner::initialize(const std::string& plugin_name, const std::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr,
                                 const rclcpp::Node::SharedPtr& node)
{
  mesh_map_ = mesh_map_ptr;
  name_ = plugin_name;
  map_frame_ = mesh_map_->mapFrame();
  node_ = node;

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());

  return true;
}

bool GAKDMeshPlanner::cancel()
{
  cancel_planning_ = true;
  return true;
}

uint32_t GAKDMeshPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
                                   const geometry_msgs::msg::PoseStamped& goal, double tolerance,
                                   std::vector<geometry_msgs::msg::PoseStamped>& plan, double& cost,
                                   std::string& message)
{
  const auto mesh = mesh_map_->mesh();
  mesh_map::Vector start_vec = mesh_map::toVector(start.pose.position);
  mesh_map::Vector goal_vec = mesh_map::toVector(goal.pose.position);

  // Find containing faces
  const lvr2::OptionalFaceHandle start_face_opt = mesh_map_->getContainingFace(start_vec, 0.4);
  const lvr2::OptionalFaceHandle goal_face_opt = mesh_map_->getContainingFace(goal_vec, 0.4);

  if (!start_face_opt)
  {
    message = "Could not find a face close enough to the given start pose";
    RCLCPP_WARN_STREAM(node_->get_logger(), "makePlan(): " << message);
    return mbf_msgs::action::GetPath::Result::INVALID_START;
  }
  if (!goal_face_opt)
  {
    message = "Could not find a face close enough to the given goal pose";
    RCLCPP_WARN_STREAM(node_->get_logger(), "makePlan(): " << message);
    return mbf_msgs::action::GetPath::Result::INVALID_GOAL;
  }

  const lvr2::FaceHandle start_face = start_face_opt.unwrap();
  const lvr2::FaceHandle goal_face = goal_face_opt.unwrap();

  // Initialize data structures
  lvr2::DenseVertexMap<float> distances(mesh->nextVertexIndex(), std::numeric_limits<float>::infinity());
  lvr2::DenseVertexMap<lvr2::VertexHandle> predecessors(mesh->nextVertexIndex(), lvr2::VertexHandle(0));
  lvr2::DenseVertexMap<bool> visited(mesh->nextVertexIndex(), false);
  lvr2::Meap<lvr2::VertexHandle, float> pq;

  // Initialize start vertices
  for (auto vH : mesh->getVerticesOfFace(start_face))
  {
    distances[vH] = (mesh->getVertexPosition(vH) - goal_vec).length();
    predecessors[vH] = vH;
    pq.insert(vH, distances[vH]);
  }

  // Greedy search
  bool goal_reached = false;
  lvr2::VertexHandle goal_vertex;
  while (!pq.isEmpty() && !cancel_planning_)
  {
    lvr2::VertexHandle current_vh = pq.popMin().key();
    if (visited[current_vh])
      continue;

    visited[current_vh] = true;

    // Check if goal face is reached
    std::vector<lvr2::FaceHandle> faces;
    mesh->getFacesOfVertex(current_vh, faces);
    for (const auto& fh : faces)
    {
      if (fh == goal_face)
      {
        goal_reached = true;
        goal_vertex = current_vh;
        break;
      }
    }
    if (goal_reached)
      break;

    // Explore neighbors
    std::vector<lvr2::VertexHandle> neighbors;
    mesh->getNeighboursOfVertex(current_vh, neighbors);
    for (const auto& neighbor : neighbors)
    {
      if (visited[neighbor])
        continue;

      float dist_to_goal = (mesh->getVertexPosition(neighbor) - goal_vec).length();
      if (dist_to_goal < distances[neighbor])
      {
        distances[neighbor] = dist_to_goal;
        predecessors[neighbor] = current_vh;
        pq.insert(neighbor, dist_to_goal);
      }
    }
  }

  if (cancel_planning_)
  {
    message = "Planning was canceled";
    RCLCPP_WARN_STREAM(node_->get_logger(), message);
    return mbf_msgs::action::GetPath::Result::CANCELED;
  }

  if (!goal_reached)
  {
    message = "No path found to the goal";
    RCLCPP_WARN_STREAM(node_->get_logger(), message);
    return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
  }

  // Backtrack to construct path
  std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>> path;
  lvr2::VertexHandle current_vh = goal_vertex;
  lvr2::FaceHandle current_face = goal_face;
  mesh_map::Vector current_pos = goal_vec;
  path.push_front({current_pos, current_face});

  while (current_vh != predecessors[current_vh])
  {
    current_vh = predecessors[current_vh];
    current_pos = mesh->getVertexPosition(current_vh);
    std::vector<lvr2::FaceHandle> vertex_faces;
    mesh->getFacesOfVertex(current_vh, vertex_faces);
    current_face = vertex_faces[0]; // Use first face for simplicity
    path.push_front({current_pos, current_face});
  }

  // Convert path to plan
  std_msgs::msg::Header header;
  header.stamp = node_->now();
  header.frame_id = map_frame_;

  cost = 0.0;
  auto it = path.begin();
  auto next_it = std::next(it);
  const auto& face_normals = mesh_map_->faceNormals();

  while (next_it != path.end())
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    float dir_length;
    pose.pose = mesh_map::calculatePoseFromPosition(it->first, next_it->first, face_normals[it->second], dir_length);
    cost += dir_length;
    plan.push_back(pose);
    ++it;
    ++next_it;
  }

  // Add goal pose
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header = header;
  goal_pose.pose = goal.pose;
  plan.push_back(goal_pose);

  // Publish path
  nav_msgs::msg::Path path_msg;
  path_msg.poses = plan;
  path_msg.header = header;
  path_pub_->publish(path_msg);

  RCLCPP_INFO_STREAM(node_->get_logger(), "Path length: " << cost << "m");
  return mbf_msgs::action::GetPath::Result::SUCCESS;
}

}  // namespace gakd_mesh_planner