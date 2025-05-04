#include <lvr2/geometry/Handles.hpp>
#include <lvr2/util/Meap.hpp>
#include <mesh_map/util.h>
#include <pluginlib/class_list_macros.hpp>
#include <random>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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

// Type definitions for genetic algorithm
typedef std::vector<double> State;
typedef std::vector<State> Trajectory;
typedef std::vector<double> Control;
typedef std::vector<Control> ControlSequence;

State dynamics(const State& x, const Control& u, double dt)
{
  return {x[0] + u[0] * cos(x[3]) * dt,
          x[1] + u[0] * sin(x[3]) * dt,
          x[2],
          x[3] + u[1] * dt};
}

Control random_control()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  const double min_velocity = -1.0;
  const double max_velocity = 5.0;
  const double min_omega = -1.0;
  const double max_omega = 3.0;
  std::uniform_real_distribution<> dis_velocity(min_velocity, max_velocity);
  std::uniform_real_distribution<> dis_steering(min_omega, max_omega);
  return {dis_velocity(gen), dis_steering(gen)};
}

std::vector<ControlSequence> generate_population(int size, int horizon)
{
  std::vector<ControlSequence> population(size, ControlSequence(horizon));
  for (auto& individual : population)
  {
    for (auto& controls : individual)
    {
      controls = random_control();
    }
  }
  return population;
}

std::pair<ControlSequence, ControlSequence> select_parents(const std::vector<ControlSequence>& population,
                                                          const std::vector<double>& fitnesses)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, population.size() - 1);
  int tournament_size = 3;
  std::vector<std::pair<ControlSequence, double>> tournament;
  for (int i = 0; i < tournament_size; ++i)
  {
    int idx = dis(gen);
    tournament.push_back({population[idx], fitnesses[idx]});
  }
  std::sort(tournament.begin(), tournament.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
  return {tournament[0].first, tournament[1].first};
}

std::pair<ControlSequence, ControlSequence> crossover(const ControlSequence& parent1, const ControlSequence& parent2)
{
  int horizon = parent1.size();
  ControlSequence offspring1, offspring2;
  offspring1.reserve(horizon);
  offspring2.reserve(horizon);
  for (int i = 0; i < horizon; ++i)
  {
    if (rand() % 2 == 0)
    {
      offspring1.push_back(parent1[i]);
      offspring2.push_back(parent2[i]);
    }
    else
    {
      offspring1.push_back(parent2[i]);
      offspring2.push_back(parent1[i]);
    }
  }
  return {offspring1, offspring2};
}

ControlSequence mutate(ControlSequence control_sequence, double mutation_rate = 0.01)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> dis(0, 0.1);
  for (auto& control : control_sequence)
  {
    for (auto& val : control)
    {
      if (static_cast<double>(rand()) / RAND_MAX < mutation_rate)
      {
        val += dis(gen);
      }
    }
  }
  return control_sequence;
}

double cost_function(const State& current_state, const State& next_state,
                     const Control& control, const State& state_target,
                     const mesh_map::MeshMap::Ptr& mesh_map)
{
  mesh_map::Vector current_vector(current_state[0], current_state[1], current_state[2]);
  mesh_map::Vector next_vector(next_state[0], next_state[1], next_state[2]);
  mesh_map::Vector target_vector(state_target[0], state_target[1], state_target[2]);

  double dx = next_vector.x - target_vector.x;
  double dy = next_vector.y - target_vector.y;
  double dz = next_vector.z - target_vector.z;
  double distance_to_goal = std::sqrt(dx * dx + dy * dy + dz * dz);

  // double elevation_change = std::abs(next_vector.z - current_vector.z);

  // // Check if next position is valid on mesh
  // auto face_opt = mesh_map->getContainingFace(next_vector, 0.4);
  // double face_penalty = face_opt ? 0.0 : 1000.0;

  return distance_to_goal; //+ elevation_change + face_penalty;
}

ControlSequence genetic_algorithm(const State& state_init, const State& state_target,
                                 double dt, int pop_size, int generations,
                                 int time_horizon, double mutation_rate,
                                 const mesh_map::MeshMap::Ptr& mesh_map)
{
  auto population = generate_population(pop_size, time_horizon);
  std::vector<double> fitnesses(pop_size);

  for (int generation = 0; generation < generations; ++generation)
  {
    for (int i = 0; i < pop_size; ++i)
    {
      State current_state = state_init;
      double total_cost = 0.0;
      for (const auto& control : population[i])
      {
        State next_state = dynamics(current_state, control, dt);
        total_cost += cost_function(current_state, next_state, control, state_target, mesh_map);
        current_state = next_state;
      }
      fitnesses[i] = total_cost;
    }

    auto min_fitness = *std::min_element(fitnesses.begin(), fitnesses.end());
    if (min_fitness < 1e-8)
    {
      break;
    }

    std::vector<ControlSequence> new_population;
    new_population.reserve(pop_size);
    for (int i = 0; i < pop_size / 2; ++i)
    {
      auto [parent1, parent2] = select_parents(population, fitnesses);
      auto [offspring1, offspring2] = crossover(parent1, parent2);
      offspring1 = mutate(offspring1, mutation_rate);
      offspring2 = mutate(offspring2, mutation_rate);
      new_population.push_back(offspring1);
      new_population.push_back(offspring2);
    }
    population = new_population;
  }

  int best_idx = std::min_element(fitnesses.begin(), fitnesses.end()) - fitnesses.begin();
  return population[best_idx];
}

Trajectory ga_planner(const State& start_point, const State& goal_point,
  double time_horizon, const mesh_map::MeshMap::Ptr& mesh_map,
  rclcpp::Node::SharedPtr node, std::atomic_bool& cancel_planning)
{
// Initialize publisher for x_current visualization
auto point_pub = node->create_publisher<geometry_msgs::msg::PointStamped>("~/current_point", rclcpp::QoS(10));

double dt = 0.1;
int max_steps = 1000;
double goal_threshold = 0.3;
Trajectory trajectory;
State x_current = start_point;
trajectory.push_back(x_current);

// Prepare header for PointStamped messages
std_msgs::msg::Header header;
header.frame_id = mesh_map->mapFrame();

for (int step = 0; step < max_steps && !cancel_planning; ++step)
{
// Publish x_current as a PointStamped message
geometry_msgs::msg::PointStamped point_msg;
point_msg.header = header;
point_msg.header.stamp = node->now();
point_msg.point.x = x_current[0];
point_msg.point.y = x_current[1];
point_msg.point.z = x_current[2];
point_pub->publish(point_msg);

// Run genetic algorithm to get optimal control
ControlSequence optimal_control = genetic_algorithm(x_current, goal_point, dt, 100, 50,
                                   time_horizon, 0.01, mesh_map);
x_current = dynamics(x_current, optimal_control[0], dt);
trajectory.push_back(x_current);

// Calculate distance to goal
double dist = std::sqrt(std::pow(x_current[0] - goal_point[0], 2) +
       std::pow(x_current[1] - goal_point[1], 2) +
       std::pow(x_current[2] - goal_point[2], 2));

if (dist < goal_threshold)
{
// Publish final point
point_msg.header.stamp = node->now();
point_msg.point.x = x_current[0];
point_msg.point.y = x_current[1];
point_msg.point.z = x_current[2];
point_pub->publish(point_msg);
return trajectory;
}
}

RCLCPP_WARN(node->get_logger(), "Max steps reached without reaching the goal.");
return {};
}

// Helper function to calculate yaw from quaternion
double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}


uint32_t GAKDMeshPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  double tolerance,
  std::vector<geometry_msgs::msg::PoseStamped>& plan,
  double& cost,
  std::string& message)
{
const auto& mesh = mesh_map_->mesh();
mesh_map::Vector start_vec = mesh_map::toVector(start.pose.position);
mesh_map::Vector goal_vec = mesh_map::toVector(goal.pose.position);

// Prepare state for genetic algorithm
State start_state = std::vector<double>{start.pose.position.x, start.pose.position.y, start.pose.position.z,
         getYawFromQuaternion(start.pose.orientation)};
State goal_state = std::vector<double>{goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
        getYawFromQuaternion(goal.pose.orientation)};

// Run genetic algorithm planner
Trajectory traj = ga_planner(start_state, goal_state, 10, mesh_map_, node_, cancel_planning_);

if (cancel_planning_)
{
message = "Planning was canceled";
RCLCPP_WARN_STREAM(node_->get_logger(), message);
return mbf_msgs::action::GetPath::Result::CANCELED;
}

if (traj.empty())
{
message = "No path found to the goal";
RCLCPP_WARN_STREAM(node_->get_logger(), message);
return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
}

RCLCPP_INFO_STREAM(node_->get_logger(), "Path found with " << traj.size() << " points");

// Convert trajectory to plan
std_msgs::msg::Header header;
header.stamp = node_->now();
header.frame_id = map_frame_;
cost = 0.0;
mesh_map::Vector current = start_vec;
nav_msgs::msg::Path path_msg;
path_msg.header = header;

for (size_t i = 0; i < traj.size(); ++i)
{
geometry_msgs::msg::PoseStamped pose;
pose.header = header;
const auto& next_state = traj[i];

geometry_msgs::msg::Point next_point;
next_point.x = next_state[0];
next_point.y = next_state[1];
next_point.z = next_state[2];
mesh_map::Vector next = mesh_map::toVector(next_point);

// Log positions for debugging
RCLCPP_DEBUG_STREAM(node_->get_logger(), "Processing point " << i << ": Current (" << current.x << ", " << current.y << ", " << current.z << "), Next (" << next.x << ", " << next.y << ", " << next.z << ")");

// Set default pose: use position from trajectory and start pose's orientation
pose.pose.position = next_point;
pose.pose.orientation = start.pose.orientation; // Default orientation
float dir_length = (next - current).length();

cost += dir_length;
current = next;
plan.push_back(pose);
path_msg.poses.push_back(pose);
}

// Add goal pose
geometry_msgs::msg::PoseStamped goal_pose;
goal_pose.header = header;
goal_pose.pose = goal.pose;
plan.push_back(goal_pose);
path_msg.poses.push_back(goal_pose);

// Publish path
path_pub_->publish(path_msg);

RCLCPP_INFO_STREAM(node_->get_logger(), "Path length: " << cost << "m");
return mbf_msgs::action::GetPath::Result::SUCCESS;
}

}  // namespace gakd_mesh_planner