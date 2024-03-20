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

#include "kinodynamic_wavefront_planner/kinodynamic_wavefront_planner.h"
#include "kinodynamic_wavefront_planner/uniform_bspline.h"


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

  std::vector<mesh_map::Vector> path_points_2 = findMinimalCostPath(start_vec,
   goal_vec,
   [this](const mesh_map::Vector& from, const mesh_map::Vector& to) -> float {
        return this->getSteeringAngleCost(from, to);
    });
  nav_msgs::Path min_steering_path = getNavPathFromVertices(path_points_2);
  
  nav_msgs::Path cvp_path = getCvpPath(path, goal_vec, cost);

  plan = cvp_path.poses;
  path_pub.publish(cvp_path);
  
  path_pub2.publish(min_steering_path);

//   auto bsp_path = getBsplinePath(path_points_2);

//   path_pub1.publish(bsp_path);
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



float KinodynamicWavefrontPlanner::getSteeringAngleCost(const mesh_map::Vector& from,
 const mesh_map::Vector& to) {
    const auto& mesh = mesh_map->mesh();
    auto p_from = from;
    auto p_to = to;
    std::vector<double> current_state = {p_from.x, p_from.y, atan2(p_from.y, p_from.x)};
    std::vector<double> next_state = {p_to.x, p_to.y, atan2(p_to.y, p_to.x)};
    double steering_angle = calculateSteeringAngle(current_state, next_state,2,0.1);

     // Normalize steering_angle to be within [0, 2π]
    steering_angle = fmod(steering_angle + 2 * M_PI, 2 * M_PI);

    // Map the steering angle from [0, 2π] to [0, 1]
    double normalized_steering_angle = steering_angle / (2 * M_PI);
    return normalized_steering_angle;
}



struct VectorHash {
size_t operator()(const mesh_map::Vector& v) const {
return std::hash<float>()(v.x) ^ std::hash<float>()(v.y) ^ std::hash<float>()(v.z);
}
};

bool operator==(const mesh_map::Vector& a, const mesh_map::Vector& b) {
return a.x == b.x && a.y == b.y && a.z == b.z;
}

// std::vector<mesh_map::Vector> KinodynamicWavefrontPlanner::getAdjacentVertices(const mesh_map::Vector& position)
// {
//     std::vector<mesh_map::Vector> adjacentPositions;
//     const auto& mesh = mesh_map->mesh();
//     lvr2::VertexHandle vertex = mesh_map->getNearestVertexHandle(position).unwrap();

//     try
//     {
//         std::vector<lvr2::EdgeHandle> connectedEdges;
//         mesh.getEdgesOfVertex(vertex, connectedEdges);

//         for (const auto& edge : connectedEdges)
//         {
//             std::array<lvr2::VertexHandle, 2> vertices = mesh.getVerticesOfEdge(edge);
//             for (const auto& vert : vertices) {
//             if (vert != vertex) {
//                 mesh_map::Vector pos = mesh.getVertexPosition(vert);
//                 adjacentPositions.push_back(pos);
//             }
//         }
//     }
// }
// catch (const std::exception& e)
// {
//     ROS_ERROR_STREAM("Error in getAdjacentVertices: " << e.what());
// }

// return adjacentPositions;
// }

std::vector<mesh_map::Vector> KinodynamicWavefrontPlanner::getAdjacentVertices(const mesh_map::Vector& position, int pointsPerEdge)
{
    std::vector<mesh_map::Vector> adjacentPositions;
    const auto& mesh = mesh_map->mesh();
    lvr2::VertexHandle vertex = mesh_map->getNearestVertexHandle(position).unwrap();

    try
    {
        std::vector<lvr2::EdgeHandle> connectedEdges;
        mesh.getEdgesOfVertex(vertex, connectedEdges);

        for (const auto& edge : connectedEdges)
        {
            std::array<lvr2::VertexHandle, 2> vertices = mesh.getVerticesOfEdge(edge);
            mesh_map::Vector startPos = mesh.getVertexPosition(vertices[0]);
            mesh_map::Vector endPos = mesh.getVertexPosition(vertices[1]);

            // Always include vertices, calculate number of points to add between them
            int intermediatePointsCount = pointsPerEdge - 2; // Subtract vertices at the ends

            for (int i = 0; i <= intermediatePointsCount + 1; ++i)
            {
                // For the vertices and the intermediate points
                float alpha = i == 0 ? 0.0f : i == intermediatePointsCount + 1 ? 1.0f : static_cast<float>(i) / (intermediatePointsCount + 1);
                mesh_map::Vector pos = startPos * (1.0f - alpha) + endPos * alpha;

                // Prevent duplicates of the start vertex for the first edge and the end vertex for the last edge
                if (!(i == 0 && adjacentPositions.size() > 0 && adjacentPositions.back() == pos) && 
                    !(i == intermediatePointsCount + 1 && vertices[1] == vertex))
                {
                    adjacentPositions.push_back(pos);
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Error in getAdjacentVertices: " << e.what());
    }

    return adjacentPositions;
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


float KinodynamicWavefrontPlanner::calculateCostAtPosition(const mesh_map::Vector& position) {
    auto goal = position;
    
    const auto& mesh = mesh_map->mesh();
    auto goal_face = mesh_map->getContainingFace(goal, 0.4).unwrap();

    const auto& vertex_handles = mesh.getVerticesOfFace(goal_face);

    // Compute barycentric coordinates and distance
    std::array<float, 3> barycentric_coords;
    float dist;
    mesh_map->projectedBarycentricCoords(goal, goal_face, barycentric_coords, dist);

    // Compute cost
    const float cost = mesh_map->costAtPosition(vertex_handles, barycentric_coords);
    return cost;
}


void saveVectorToFile(const std::vector<Eigen::Vector3d>& vec, const std::string& filepath) {
    std::ofstream outFile(filepath);

    if (!outFile.is_open()) {
        std::cerr << "Failed to open file: " << filepath << std::endl;
        return;
    }

    for (const auto& v : vec) {
        outFile << v(0) << ", " << v(1) << ", " << v(2) << "\n"; 
    }

    outFile.close();
}


void  KinodynamicWavefrontPlanner::savePathAndNormals(const std::vector<mesh_map::Vector>& path,
  const std::string& pathFileName,
   const std::string& normalsFileName) {

    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector3d> normals;

    for (const auto& position : path) {
        auto position_ = position;
        const auto& face_normals = mesh_map->faceNormals();
        const lvr2::FaceHandle face = mesh_map->getContainingFace(position_, 0.4).unwrap();
        mesh_map::Normal normal = face_normals[face];
        
        positions.push_back(Eigen::Vector3d(position.x, position.y, position.z));
        normals.push_back(Eigen::Vector3d(normal.x, normal.y, normal.z));
    }
    saveVectorToFile(positions, pathFileName);
    saveVectorToFile(normals, normalsFileName);

}

std::vector<Eigen::Vector3d> convertPath(const std::vector<mesh_map::Vector>& path) {
    std::vector<Eigen::Vector3d> convertedPath;
    convertedPath.reserve(path.size());
    
    for (const auto& point : path) {
        Eigen::Vector3d eigenPoint(point.x, point.y, point.z);
        convertedPath.push_back(eigenPoint);
    }
    
    return convertedPath;
}

std::vector<mesh_map::Vector> convertBack(const std::vector<Eigen::Vector3d>& convertedPath) {
    std::vector<mesh_map::Vector> path;
    path.reserve(convertedPath.size());

    for (const auto& eigenPoint : convertedPath) {
        mesh_map::Vector point;
        point.x = eigenPoint[0];
        point.y = eigenPoint[1];
        point.z = eigenPoint[2];
        path.push_back(point);
    }

    return path;
}

std::vector<Eigen::Vector3d> smoothPath(const std::vector<Eigen::Vector3d>& path, 
int iterations = 5, double neighbor_weight = 0.1) {

    std::vector<Eigen::Vector3d> smoothed_path = path;
    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<Eigen::Vector3d> temp_path = smoothed_path;
        for (size_t i = 1; i < path.size() - 1; ++i) {
            const Eigen::Vector3d& prev_point = smoothed_path[i - 1];
            const Eigen::Vector3d& next_point = smoothed_path[i + 1];
            temp_path[i] = (1 - 2 * neighbor_weight) * smoothed_path[i] + neighbor_weight * (prev_point + next_point);
        }
        smoothed_path = std::move(temp_path);
    }
    return smoothed_path;
}

std::vector<mesh_map::Vector> KinodynamicWavefrontPlanner::findMinimalCostPath(
     const mesh_map::Vector& original_start,
     const mesh_map::Vector& original_goal,
     std::function<double(const mesh_map::Vector&, const mesh_map::Vector&)> cost_function) {

  const auto& mesh = mesh_map->mesh();
  const auto& vertex_costs = mesh_map->vertexCosts();

  mesh_map::Vector start = original_start;
  mesh_map::Vector goal = original_goal;

  const auto& start_face = mesh_map->getContainingFace(start, 0.4).unwrap();
  const auto& goal_face = mesh_map->getContainingFace(goal, 0.4).unwrap();
  
  std::array<mesh_map::Vector, 3> goal_positions = mesh.getVertexPositionsOfFace(goal_face);

  lvr2::Meap<mesh_map::Vector, float> pq;
  pq.insert(start, 0);


  std::unordered_map<mesh_map::Vector, bool, VectorHash> visited;

  std::unordered_map<mesh_map::Vector, mesh_map::Vector, VectorHash> predecessors;

  while (!pq.isEmpty()) {
    auto pair = pq.popMin();
    auto current_pos = pair.key(); 
    visited[current_pos] = true;


    for (const auto& goal_pos : goal_positions) {
        if (current_pos == goal_pos) {
            ROS_INFO(">>>>>>>>>>>>Wavefront reached the goal!");
            goto reconstruct_path;
        }
    }

    std::vector<mesh_map::Vector> neighbors = getAdjacentVertices(current_pos,20);
    for (const auto& neighbor : neighbors) {
      if (visited[neighbor]) continue; 

      if (!pq.containsKey(neighbor)) {
        lvr2::VertexHandle neighbor_vh = mesh_map->getNearestVertexHandle(neighbor).unwrap();
        float neighbour_cost =  vertex_costs[neighbor_vh] + cost_function(current_pos, neighbor);
        pq.insert(neighbor, neighbour_cost);
        predecessors.emplace(neighbor, current_pos);
      }
    }
  }

reconstruct_path:
  std::vector<mesh_map::Vector> path;
  path.push_back(goal);
  for (const auto& goal_pos : goal_positions) {
      auto it = predecessors.find(goal_pos);
      if (it != predecessors.end()) {
          auto pos = goal_pos;
          
          while (it != predecessors.end()) {
              path.push_back(pos);
              pos = it->second;
              it = predecessors.find(pos);
          }
          
          break;
      }
  }

  std::reverse(path.begin(), path.end());
//   savePathAndNormals(path,"path_data.txt","normal_data.txt");

 std::vector<Eigen::Vector3d> eigenPath =  convertPath(path);
 std::vector<Eigen::Vector3d> smooth_path = smoothPath(eigenPath);

 std::vector<mesh_map::Vector> final_path =  convertBack(smooth_path);

  return final_path;
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

nav_msgs::Path KinodynamicWavefrontPlanner::getNavPathFromVertices(const std::vector<mesh_map::Vector> &path) {
    const auto& mesh = mesh_map->mesh();
    const auto& vertex_normals = mesh_map->vertexNormals();

    nav_msgs::Path nav_path;
    nav_path.header.frame_id = mesh_map->mapFrame();
    nav_path.header.stamp = ros::Time::now();

    for (size_t i = 0; i < path.size(); ++i) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = nav_path.header.frame_id;
        pose_stamped.header.stamp = ros::Time::now();

        auto position = path[i];
        pose_stamped.pose.position.x = position.x;
        pose_stamped.pose.position.y = position.y;
        pose_stamped.pose.position.z = position.z;

        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;

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
