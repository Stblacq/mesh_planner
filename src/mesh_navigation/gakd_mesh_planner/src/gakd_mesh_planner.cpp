#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <functional>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <lvr2/geometry/Handles.hpp>
#include <lvr2/util/Meap.hpp>

#include <mbf_msgs/GetPathResult.h>
#include <mesh_map/util.h>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>

#include "gakd_mesh_planner/gakd_mesh_planner.h"


PLUGINLIB_EXPORT_CLASS(gakd_mesh_planner::GakdMeshPlanner, mbf_mesh_core::MeshPlanner);

namespace gakd_mesh_planner
{

GakdMeshPlanner::GakdMeshPlanner() : WaveFrontPlanner()
{
}

GakdMeshPlanner::~GakdMeshPlanner()
{
}


uint32_t GakdMeshPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                                               const geometry_msgs::PoseStamped& goal,
                                               double tolerance, 
                                               std::vector<geometry_msgs::PoseStamped>& plan, 
                                               double& cost,
                                               std::string& message)
{
    ROS_INFO("Starting wave front propagation.");
    ROS_INFO_STREAM("Map Frame ID: " << mesh_map->mapFrame());
    const auto& mesh = mesh_map->mesh();
    std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>> path;

    mesh_map::Vector goal_vec = mesh_map::toVector(goal.pose.position);
    mesh_map::Vector start_vec = mesh_map::toVector(start.pose.position);

    // uint32_t outcome = waveFrontPropagation(goal_vec, start_vec, path);
    // nav_msgs::Path cvp_path = getCvpPath(path, goal_vec, cost);

    auto start_state = {start.pose.position.x, start.pose.position.y, start.pose.position.z, tf::getYaw(start.pose.orientation)};
    auto goal_state = {goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, tf::getYaw(goal.pose.orientation)};


    Trajectory traj = ga_planner(start_state,goal_state, 10);

    ROS_INFO_STREAM("Converged...");

    auto current=start_vec;
    nav_msgs::Path gakd_path;
    gakd_path.header.stamp = ros::Time::now();
    gakd_path.header.frame_id = mesh_map->mapFrame();;

    for (size_t i = 0; i < traj.size(); ++i) {
        geometry_msgs::PoseStamped pose; 
        const auto& next_state = traj[i];

        geometry_msgs::Point next_point; 
        next_point.x = next_state[0]; 
        next_point.y = next_state[1];
        next_point.z = start_vec.z;

        mesh_map::Vector next = mesh_map::toVector(next_point); 

        const lvr2::FaceHandle& state_face = mesh_map->getContainingFace(current, 0.4).unwrap();

        auto normal = mesh_map->faceNormals()[state_face];

        float dir_length = (next - current).length();

        pose.pose = mesh_map::calculatePoseFromPosition(current, next, normal, dir_length);
        cost += dir_length;
        current = next; 
        
        gakd_path.poses.push_back(pose);
    }

   // Add goal point to the path
    geometry_msgs::Point goal_point; goal_point.x = goal_vec.x; goal_point.y = goal_vec.y;goal_point.z = goal_vec.z;
    mesh_map::Vector goal_vector = mesh_map::toVector(goal_point);
    float final_dir_length = (goal_vector - current).length();
    const lvr2::FaceHandle& goal_face = mesh_map->getContainingFace(goal_vector, 0.4).unwrap();
    auto goal_normal = mesh_map->faceNormals()[goal_face];

    // Create the final pose for the goal
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose = mesh_map::calculatePoseFromPosition(current, goal_vector, goal_normal, final_dir_length);
    cost += final_dir_length;

    // Add the goal pose to the gakd_path
    gakd_path.poses.push_back(goal_pose);

    path_pub.publish(gakd_path);
    // path_pub1.publish(gakd_path);

    plan = gakd_path.poses;
    mesh_map->publishVertexCosts(potential, "Potential");
    if (publish_vector_field)
    {
        mesh_map->publishVectorField("vector_field", vector_map, publish_face_vectors);
    }

    ROS_INFO_STREAM("Path length: " << cost << "m");

    return mbf_msgs::GetPathResult::SUCCESS;

}

nav_msgs::Path GakdMeshPlanner::getCvpPath(std::list<std::pair<mesh_map::Vector, 
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


boost::optional<mesh_map::Vector>  GakdMeshPlanner::calculateDirectionAtPosition(const mesh_map::Vector& position) {
    auto goal = position;
    
    const auto& mesh = mesh_map->mesh();
    auto goal_face = mesh_map->getContainingFace(goal, 0.4).unwrap();

    const auto& vertex_handles = mesh.getVerticesOfFace(goal_face);

    // Compute barycentric coordinates
    std::array<float, 3> barycentric_coords;
    float dist;
    mesh_map->projectedBarycentricCoords(goal, goal_face, barycentric_coords, dist);

    // Compute Direction
    boost::optional<mesh_map::Vector>  direction =  mesh_map->directionAtPosition(vector_map,vertex_handles,barycentric_coords);
    return direction;
}


Trajectory GakdMeshPlanner::ga_planner(const State& start_point, const State& goal_point, double time_horizon =10) {
    double dt = 0.1;
    int max_steps = 100000;
    double goal_threshold = 1.3;
    Trajectory trajectory;
    State x_current = start_point;

    trajectory.push_back(x_current);

    for (int step = 0; step < max_steps; ++step) {
        clock_t start_time = clock();
        ControlSequence optimal_control = genetic_algorithm(x_current, goal_point, dt, 100, 50, time_horizon, 0.01);

        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp = ros::Time::now();
        point_msg.header.frame_id = mesh_map->mapFrame();
        point_msg.point.x = x_current[0];
        point_msg.point.y = x_current[1];
        point_msg.point.z = x_current[2];
        realtime_path_pub.publish(point_msg);

        geometry_msgs::Twist control_msg;
        control_msg.linear.x = optimal_control[0][0];
        control_msg.angular.z = optimal_control[0][1];
        cmd_vel_pub.publish(control_msg);

        ros::Duration(dt).sleep();

        x_current[0] = current_state.position.x;
        x_current[1] = current_state.position.y;
        x_current[2] = current_state.position.z;
        x_current[3] = tf::getYaw(current_state.orientation);

        trajectory.push_back(x_current);

        double dist = sqrt(pow(x_current[0] - goal_point[0], 2) +
                           pow(x_current[1] - goal_point[1], 2) +
                           pow(x_current[2] - goal_point[2], 2));

        if (dist < goal_threshold) {
            return trajectory;
        }
    }

    ROS_WARN("Max steps reached without reaching the goal.");
    return {};
}

// Trajectory GakdMeshPlanner::ga_planner(const State& start_point, const State& goal_point, double time_horizon = 10)
// {
//     double dt = 0.1;
//     int max_steps = 100000;
//     double goal_threshold = 0.3;
//     Trajectory trajectory;
//     State x_current = start_point;

//     trajectory.push_back(x_current);

//     for (int step = 0; step < max_steps; ++step) {
//         clock_t start_time = clock();
//         ControlSequence optimal_control = genetic_algorithm(x_current, goal_point, dt, 100, 50, time_horizon, 0.01);
//         x_current = dynamics(x_current, optimal_control[0], dt);

//         mesh_map::Vector current_vector; 
//         current_vector.x = x_current[0]; 
//         current_vector.y = x_current[1]; 
//         current_vector.z = x_current[2];

//         auto projected_x_error = projectToFaceAndDistance(current_vector);
//         auto projected_x = projected_x_error->first;

//         // Update x_current without redeclaring it
//         x_current[0] = projected_x.x;
//         x_current[1] = projected_x.y;
//         x_current[2] = projected_x.z;

//         trajectory.push_back(x_current);
        
//         geometry_msgs::PointStamped point_msg;
//         point_msg.header.stamp = ros::Time::now();
//         point_msg.header.frame_id = mesh_map->mapFrame();
//         point_msg.point.x = x_current[0];
//         point_msg.point.y = x_current[1];
//         point_msg.point.z = x_current[2];
//         realtime_path_pub.publish(point_msg);

//         double dist = sqrt(pow(x_current[0] - goal_point[0], 2) + 
//                            pow(x_current[1] - goal_point[1], 2) +
//                            pow(x_current[2] - goal_point[2], 2));

//         if (dist < goal_threshold) {
//             return trajectory;
//         }
//     }

//     cout << "Max steps reached without reaching the goal." << endl;
//     return {};
// }

State GakdMeshPlanner::dynamics(const State& x, const Control& u, double dt) {
    return {x[0] + u[0] * cos(x[3]) * dt,
     x[1] + u[0] * sin(x[3]) * dt, 
     x[2],
     x[3] + u[1] * dt};
}

Control GakdMeshPlanner::random_control() {
    random_device rd;
    mt19937 gen(rd());
    
    const double min_velocity = -1;
    const double max_velocity = 5.0;
    const double min_omega = -1;
    const double max_omega = 3;

    uniform_real_distribution<> dis_velocity(min_velocity, max_velocity);
    uniform_real_distribution<> dis_steering(min_omega, max_omega);
    
    vector<double> control(2);
    control[0] = dis_velocity(gen);
    control[1] = dis_steering(gen);
    
    return control;
}


vector<ControlSequence> GakdMeshPlanner::generate_population(int size, int horizon) {
    vector<vector<vector<double>>> population(size, vector<vector<double>>(horizon));
    for (auto& individual : population) {
        for (auto& controls : individual) {
            controls = random_control();
        }
    }
    return population;
}

std::pair<ControlSequence, ControlSequence> GakdMeshPlanner::select_parents(const vector<ControlSequence>& population, 
                                                                        const vector<double>& fitnesses) {
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, population.size() - 1);
    int tournament_size = 3;

    vector<pair<vector<vector<double>>, double>> tournament;
    for (int i = 0; i < tournament_size; ++i) {
        int idx = dis(gen);
        tournament.push_back({population[idx], fitnesses[idx]});
    }

    sort(tournament.begin(), tournament.end(), [](const auto& a, const auto& b) {
        return a.second < b.second; // Sort by fitness (ascending)
    });

    return {tournament[0].first, tournament[1].first};
}

std::pair<ControlSequence, ControlSequence> GakdMeshPlanner::crossover(
    const ControlSequence& parent1, const ControlSequence& parent2) {

    int horizon = parent1.size();
    vector<vector<double>> offspring1, offspring2;

    for (int i = 0; i < horizon; ++i) {
        if (rand() % 2 == 0) {
            offspring1.push_back(parent1[i]);
            offspring2.push_back(parent2[i]);
        } else {
            offspring1.push_back(parent2[i]);
            offspring2.push_back(parent1[i]);
        }
    }

    return {offspring1, offspring2};
}


ControlSequence GakdMeshPlanner::mutate(ControlSequence control_sequence, double mutation_rate = 0.01) {
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<> dis(0, 0.1);

    for (auto& control : control_sequence) {
        for (auto& val : control) {
            if (rand() / double(RAND_MAX) < mutation_rate) {
                val += dis(gen);
            }
        }
    }
    return control_sequence;
}

ControlSequence GakdMeshPlanner::genetic_algorithm(const State& state_init, const State& state_target,
                                                   double dt, int pop_size = 100, int generations = 50,
                                                   int time_horizon = 10, double mutation_rate = 0.01) {

    vector<vector<vector<double>>> population = generate_population(pop_size, time_horizon);
    vector<double> fitnesses(pop_size);

    for (int generation = 0; generation < generations; ++generation) {
        for (int i = 0; i < pop_size; ++i) {
            vector<double> current_state = state_init;
            double total_cost = 0.0;

            for (const auto& control : population[i]) {
                vector<double> next_state = dynamics(current_state, control, dt);
                total_cost += cost_function(current_state, next_state, control, state_target);
                current_state = next_state;
            }
            fitnesses[i] = total_cost;
        }

        auto min_fitness = *min_element(fitnesses.begin(), fitnesses.end());
        if (min_fitness < 1e-8) {
            break;
        }

        vector<vector<vector<double>>> new_population;

        for (int i = 0; i < pop_size / 2; ++i) {
            auto [parent1, parent2] = select_parents(population, fitnesses);
            auto [offspring1, offspring2] = crossover(parent1, parent2);
            offspring1 = mutate(offspring1, mutation_rate);
            offspring2 = mutate(offspring2, mutation_rate);
            new_population.push_back(offspring1);
            new_population.push_back(offspring2);
        }

        population = new_population;
    }

    int best_idx = min_element(fitnesses.begin(), fitnesses.end()) - fitnesses.begin();
    return population[best_idx];
}

float GakdMeshPlanner::computeDirectionCost(const mesh_map::Vector& current, const mesh_map::Vector& next) {
    boost::optional<mesh_map::Vector> mesh_dir_opt = calculateDirectionAtPosition(current);
    boost::optional<mesh_map::Vector> next_mesh_dir_opt = calculateDirectionAtPosition(next);
    
    if (!mesh_dir_opt.is_initialized() && next_mesh_dir_opt.is_initialized()) {
        return std::sqrt(std::pow(next.x - current.x, 2) + 
                         std::pow(next.y - current.y, 2) + 
                         std::pow(next.z - current.z, 2));
    }
    
    if (!mesh_dir_opt.is_initialized()) {
        return std::numeric_limits<float>::infinity();
    }

    mesh_map::Vector neighbor_dir = (next - current).normalized();
    mesh_map::Vector mesh_dir = mesh_dir_opt.get().normalized();

    // Compute ideal next position
    mesh_map::Vector ideal_next = current + mesh_dir;
    mesh_map::Vector diff = next - ideal_next;
    float direction_cost = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z) / 2.0f;

    return direction_cost;
}
boost::optional<std::pair<std::vector<mesh_map::Vector>, mesh_map::Vector>> GakdMeshPlanner::findClosestFace(const mesh_map::Vector &position)
{
    const auto& mesh = mesh_map->mesh();
    const auto& face_normals = mesh_map->faceNormals();

    const auto& vH_opt = mesh_map->getNearestVertexHandle(position);

    if (!vH_opt)
    {
        ROS_FATAL_STREAM("Could not find the nearest vertex for position: " << position);
        return boost::none;
    }

    auto vH = vH_opt.unwrap();
    
    std::vector<lvr2::FaceHandle> faces;
    mesh.getFacesOfVertex(vH, faces);
    
    if (faces.empty())
    {
        ROS_ERROR_STREAM("No faces found for vertex: " << vH);
        return boost::none;
    }

    float min_distance = std::numeric_limits<float>::max();
    std::vector<mesh_map::Vector> closest_face_vertices;
    mesh_map::Vector closest_face_normal;

    for (const auto &fH : faces)
    {
        auto vertex_handles = mesh.getVerticesOfFace(fH);
        
        std::array<mesh_map::Vector, 3> vertices;
        for (size_t i = 0; i < 3; i++)
        {
            vertices[i] = mesh.getVertexPosition(vertex_handles[i]);
        }

        float face_dist = (vertices[0] - position).length2() +
                          (vertices[1] - position).length2() +
                          (vertices[2] - position).length2();

        if (face_dist < min_distance)
        {
            min_distance = face_dist;
            closest_face_normal = face_normals[fH];
            closest_face_vertices = {vertices[0], vertices[1], vertices[2]};
        }
    }

    return std::make_pair(closest_face_vertices, closest_face_normal);
}

boost::optional<std::pair<mesh_map::Vector, float>> GakdMeshPlanner::projectToFaceAndDistance(const mesh_map::Vector& position) {
    auto closestFace = findClosestFace(position);
    if (!closestFace) {
        return boost::none;
    }

    const auto& vertices = closestFace->first;
    const auto& normal = closestFace->second;

    mesh_map::Vector v0 = vertices[0];
    mesh_map::Vector v1 = vertices[1];
    mesh_map::Vector v2 = vertices[2];

    mesh_map::Vector edge1 = v1 - v0;
    mesh_map::Vector edge2 = v2 - v0;
    mesh_map::Vector pointToV0 = position - v0;
    
    float d = pointToV0.dot(normal) / normal.length();
    mesh_map::Vector projectedPoint = position - normal * d;

    float signedDistance = pointToV0.dot(normal) / normal.length();

    return std::make_pair(projectedPoint, signedDistance);
}


// double GakdMeshPlanner::cost_function(const State& current_state, const State& next_state, 
//                                       const Control& control, const State& state_target) {
//     mesh_map::Vector current_vector;
//     current_vector.x = current_state[0];
//     current_vector.y = current_state[1];
//     current_vector.z = current_state[2];

//     mesh_map::Vector next_vector;
//     next_vector.x = next_state[0];
//     next_vector.y = next_state[1];
//     next_vector.z = next_state[2];

//     auto cost = computeDirectionCost(current_vector,next_vector);

//     return cost ;
// }

double GakdMeshPlanner::cost_function(const State& current_state, 
    const State& next_state, 
    const Control& control, 
    const State& state_target) {
// Convert the current state to a vector.
    mesh_map::Vector current_vector;
    current_vector.x = current_state[0];
    current_vector.y = current_state[1];
    current_vector.z = current_state[2];

    // Convert the next state to a vector.
    mesh_map::Vector next_vector;
    next_vector.x = next_state[0];
    next_vector.y = next_state[1];
    next_vector.z = next_state[2];

    // Convert the target (goal) state to a vector.
    mesh_map::Vector target_vector;
    target_vector.x = state_target[0];
    target_vector.y = state_target[1];
    target_vector.z = state_target[2];

    // Compute the Euclidean distance from the next state to the goal.
    double dx = next_vector.x - target_vector.x;
    double dy = next_vector.y - target_vector.y;
    double dz = next_vector.z - target_vector.z;
    double distance_to_goal = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Compute the absolute change in elevation from current to next.
    double elevation_change = std::abs(next_vector.z - current_vector.z);

    // Combine both terms to form the final cost.
    double cost = distance_to_goal + elevation_change;
    return cost;
}


} /* namespace gakd_mesh_planner */
