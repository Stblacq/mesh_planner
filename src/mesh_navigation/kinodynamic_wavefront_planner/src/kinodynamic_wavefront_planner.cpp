#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <functional>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <lvr2/geometry/Handles.hpp>
#include <lvr2/util/Meap.hpp>

#include <mbf_msgs/GetPathResult.h>
#include <mesh_map/util.h>
#include <pluginlib/class_list_macros.h>

#include "geometrycentral/surface/flip_geodesics.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/mesh_graph_algorithms.h"

#include "kinodynamic_wavefront_planner/kinodynamic_wavefront_planner.h"
#include "kinodynamic_wavefront_planner/uniform_bspline.h"


using namespace geometrycentral;
using namespace geometrycentral::surface;

PLUGINLIB_EXPORT_CLASS(kinodynamic_wavefront_planner::KinodynamicWavefrontPlanner, mbf_mesh_core::MeshPlanner);

namespace kinodynamic_wavefront_planner
{
KinodynamicWavefrontPlanner::KinodynamicWavefrontPlanner() : WaveFrontPlanner()
{
}

KinodynamicWavefrontPlanner::~KinodynamicWavefrontPlanner()
{
}


uint32_t KinodynamicWavefrontPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                    double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost,
                                    std::string& message)
{
  const auto& mesh = mesh_map->mesh();
  std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>> path;

  // mesh_map->combineVertexCosts(); // TODO should be outside the planner

  ROS_INFO("start wave front propagation.");

  mesh_map::Vector goal_vec = mesh_map::toVector(goal.pose.position);
  mesh_map::Vector start_vec = mesh_map::toVector(start.pose.position);

  uint32_t outcome = waveFrontPropagation(goal_vec, start_vec, path);

  std::vector<lvr2::VertexHandle> path_points_2 = findMinimalCostPath(start_vec,
   goal_vec,
   [this](const lvr2::VertexHandle& from, const lvr2::VertexHandle& to) -> float {
        return this->getSteeringAngleCost(from, to);
    });
  nav_msgs::Path min_steering_path = getNavPathFromVertices(path_points_2);
  
  nav_msgs::Path cvp_path = getCvpPath(path, goal_vec, cost);

  plan = cvp_path.poses;
  path_pub.publish(cvp_path);
  
  path_pub2.publish(min_steering_path);

  auto bsp_path = getBsplinePath(path_points_2);

  path_pub1.publish(bsp_path);
  mesh_map->publishVertexCosts(potential, "Potential");
  ROS_INFO_STREAM("Path length: " << cost << "m");

  if (publish_vector_field)
  {
    mesh_map->publishVectorField("vector_field", vector_map, publish_face_vectors);
  }

  return outcome;
}

float KinodynamicWavefrontPlanner::getKinodynamicCost(const lvr2::VertexHandle& from,
 const lvr2::VertexHandle& to) {
    return 0;
}


float KinodynamicWavefrontPlanner::getSteeringAngleCost(const lvr2::VertexHandle& from,
 const lvr2::VertexHandle& to) {
    const auto& mesh = mesh_map->mesh();
    auto p_from = mesh.getVertexPosition(from);
    auto p_to = mesh.getVertexPosition(to);
    std::vector<double> current_state = {p_from.x, p_from.y, atan2(p_from.y, p_from.x)};
    std::vector<double> next_state = {p_to.x, p_to.y, atan2(p_to.y, p_to.x)};
    double steering_angle = calculateSteeringAngle(current_state, next_state,2,0.1);

     // Normalize steering_angle to be within [0, 2π]
    steering_angle = fmod(steering_angle + 2 * M_PI, 2 * M_PI);

    // Map the steering angle from [0, 2π] to [0, 1]
    double normalized_steering_angle = steering_angle / (2 * M_PI);
    return normalized_steering_angle;
}



std::vector<lvr2::VertexHandle> KinodynamicWavefrontPlanner::getAdjacentVertices(const lvr2::VertexHandle& vertex)
{
    std::vector<lvr2::VertexHandle> adjacentVertices;
    const auto& mesh = mesh_map->mesh();

    try
    {
        std::vector<lvr2::EdgeHandle> connectedEdges;
        mesh.getEdgesOfVertex(vertex, connectedEdges);

        for (auto& edge : connectedEdges)
        {
            std::array<lvr2::VertexHandle, 2> vertices = mesh.getVerticesOfEdge(edge);

            if (vertices[0] != vertex)
            {
                adjacentVertices.push_back(vertices[0]);
            }
            else if (vertices[1] != vertex)
            {
                adjacentVertices.push_back(vertices[1]);
            }
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Error in getAdjacentVertices: " << e.what());
    }

    return adjacentVertices;
}

nav_msgs::Path KinodynamicWavefrontPlanner::getCvpPath(std::list<std::pair<mesh_map::Vector, 
lvr2::FaceHandle>>& path, const mesh_map::Vector& goal_vec, double& cost) {
    path.reverse();

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = mesh_map->mapFrame();

    cost = 0;
    float dir_length;
    nav_msgs::Path path_msg;
    path_msg.header = header;

    if (!path.empty()) {
        mesh_map::Vector vec = path.front().first;
        lvr2::FaceHandle fH = path.front().second;
        path.pop_front();

        const auto& face_normals = mesh_map->faceNormals();
        for (auto& next : path) {
            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose = mesh_map::calculatePoseFromPosition(vec, next.first, face_normals[fH], dir_length);
            cost += dir_length;
            vec = next.first;
            fH = next.second;
            path_msg.poses.push_back(pose);
        }

        geometry_msgs::PoseStamped last_pose;
        last_pose.header = header;
        last_pose.pose = mesh_map::calculatePoseFromPosition(vec, goal_vec, face_normals[fH], dir_length);
        cost += dir_length;
        path_msg.poses.push_back(last_pose);
    }
    return path_msg;
}

std::vector<lvr2::VertexHandle> KinodynamicWavefrontPlanner::findMinimalCostPath(
     const mesh_map::Vector& original_start,
     const mesh_map::Vector& original_goal,
     std::function<double(const lvr2::VertexHandle&, const lvr2::VertexHandle&)> cost_function) {

  // Access the mesh and the pre-computed vertex costs
  const auto& mesh = mesh_map->mesh();
  const auto& vertex_costs = mesh_map->vertexCosts();

  mesh_map::Vector start = original_start;
  mesh_map::Vector goal = original_goal;
  // Find the containing faces of start and goal, with error handling for optional values
  const auto& start_face = mesh_map->getContainingFace(start, 0.4).unwrap();
  const auto& goal_face = mesh_map->getContainingFace(goal, 0.4).unwrap();
  
  std::array<lvr2::VertexHandle, 3> goal_vertices = mesh.getVerticesOfFace(goal_face);

  // Initialize the priority queue
  lvr2::Meap<lvr2::VertexHandle, float> pq;

  // Populate the priority queue with the vertices of the starting face and their costs
  for (auto vH : mesh.getVerticesOfFace(start_face)) {
    pq.insert(vH, vertex_costs[vH]);
  }

  // To keep track of visited vertices to avoid revisiting
  std::unordered_map<lvr2::VertexHandle, bool> visited;

  // To keep track of the path
  std::unordered_map<lvr2::VertexHandle, lvr2::VertexHandle> predecessors;

  while (!pq.isEmpty()) {
    auto pair = pq.popMin();
    auto current_vh = pair.key(); 
    // Mark the current vertex as visited
    visited[current_vh] = true;

    // Check if the current vertex is one of the goal vertices
    if (current_vh == goal_vertices[0] || current_vh == goal_vertices[1] || current_vh == goal_vertices[2]) {
      ROS_INFO(">>>>>>>>>>>>Wavefront reached the goal!");
      break; // Goal reached
    }

    // Explore neighbors
    std::vector<lvr2::VertexHandle> neighbors = getAdjacentVertices(current_vh);
    for (auto neighbor : neighbors) {
      if (visited[neighbor]) continue; // Skip visited neighbors

      if (!pq.containsKey(neighbor)) {
        float neighbour_cost =  vertex_costs[neighbor] + cost_function(current_vh,neighbor);
        pq.insert(neighbor, neighbour_cost);
        predecessors.emplace(neighbor, current_vh);
      }
    }
  }

  // Reconstruct the path from the goal to the start

std::vector<lvr2::VertexHandle> path;

// Start from a goal vertex that has been reached and is in predecessors
for (auto gv : goal_vertices) {
    auto it = predecessors.find(gv);
    if (it != predecessors.end()) {
        auto v = gv;
        
        while (it != predecessors.end()) {
            path.push_back(v); // Add the current vertex to the path
            v = it->second; // Move to the predecessor of the current vertex
            it = predecessors.find(v); // Update iterator to the predecessor
        }
        
        // After the loop, 'v' will be the start vertex, add it if not already included
        if (path.empty() || path.back() != v) {
            path.push_back(v);
        }
        
        break;
    }
}

std::reverse(path.begin(), path.end());

return path;
}

Eigen::Vector3d KinodynamicWavefrontPlanner::getPosition(const lvr2::VertexHandle& vertex_handle) {
    const auto& mesh = mesh_map->mesh();
    auto position = mesh.getVertexPosition(vertex_handle);
    return Eigen::Vector3d(position.x, position.y, position.z);
}

nav_msgs::Path KinodynamicWavefrontPlanner::getBsplinePath(const std::vector<lvr2::VertexHandle>& path) {

    Eigen::MatrixXd points(3, path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        Eigen::Vector3d pos = getPosition(path[i]);
        points.col(i) = pos;
    }

    int order = 3;
    double interval = 1.0;

    UniformBspline bspline(points, order, interval);

    nav_msgs::Path nav_path;
    nav_path.header.frame_id = mesh_map->mapFrame();
    nav_path.header.stamp = ros::Time::now();

    for (int i = 0; i <= 100; ++i) {
        double u = bspline.getKnot().minCoeff() + (bspline.getKnot().maxCoeff() - bspline.getKnot().minCoeff()) * i / 100.0;
        Eigen::VectorXd point = bspline.evaluateDeBoor(u);
        
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = nav_path.header.frame_id;
        pose_stamped.header.stamp = ros::Time::now();

        pose_stamped.pose.position.x = point[0];
        pose_stamped.pose.position.y = point[1];
        pose_stamped.pose.position.z = point[2];

        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;

        nav_path.poses.push_back(pose_stamped);

    }
    return nav_path;
}

nav_msgs::Path KinodynamicWavefrontPlanner::getNavPathFromVertices(const std::vector<lvr2::VertexHandle> &path) {
    const auto& mesh = mesh_map->mesh();
    const auto& vertex_normals = mesh_map->vertexNormals();

    nav_msgs::Path nav_path;
    nav_path.header.frame_id = mesh_map->mapFrame();
    nav_path.header.stamp = ros::Time::now();

    for (size_t i = 0; i < path.size(); ++i) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = nav_path.header.frame_id;
        pose_stamped.header.stamp = ros::Time::now();

        auto position = mesh.getVertexPosition(path[i]);
        pose_stamped.pose.position.x = position.x;
        pose_stamped.pose.position.y = position.y;
        pose_stamped.pose.position.z = position.z;

        if (i < path.size() - 1) {
            auto nextPosition = mesh.getVertexPosition(path[i + 1]);
            auto normal = vertex_normals[path[i+1]];
            auto pose = mesh_map::calculatePoseFromPosition(position, nextPosition, normal);
            pose_stamped.pose.orientation = pose.orientation; 
            } else {
            // Handle the last point's orientation, 
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;
        }
        nav_path.poses.push_back(pose_stamped);
    }

    return nav_path;
}



double KinodynamicWavefrontPlanner::calculateSteeringAngle(const std::vector<double>& current_state,
                              const std::vector<double>& next_state,
                              double L = 2.0, double dt = 0.1) {
    double dx = next_state[0] - current_state[0];
    double dy = next_state[1] - current_state[1];
    double trajectory_angle = atan2(dy, dx);
    double orientation_diff = trajectory_angle - current_state[2];
    orientation_diff = fmod(orientation_diff + M_PI, 2 * M_PI) - M_PI;
    double distance = sqrt(dx*dx + dy*dy);
    double radius_of_curvature = orientation_diff != 0 ? distance / (2 * sin(orientation_diff / 2)) : std::numeric_limits<double>::infinity();
    double delta = radius_of_curvature != std::numeric_limits<double>::infinity() ? atan(L / radius_of_curvature) : 0;
    return delta;
}

} /* namespace kinodynamic_wavefront_planner */
