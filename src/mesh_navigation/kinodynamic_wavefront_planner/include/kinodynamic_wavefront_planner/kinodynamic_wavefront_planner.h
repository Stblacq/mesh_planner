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


  struct State {
      float x, y, z;
      float qw, qx, qy, qz;

      // Default constructor
      State() : x(0), y(0), z(0), qw(0), qx(0), qy(0), qz(0) {}

      // Constructor with parameters
      State(float x, float y, float z, float qw, float qx, float qy, float qz)
      : x(x), y(y), z(z), qw(qw), qx(qx), qy(qy), qz(qz) {}

      // Equality operator to compare two State objects
      bool operator==(const State& other) const {
          return x == other.x && y == other.y && z == other.z;
              //  && qw == other.qw && qx == other.qx && qy == other.qy && qz == other.qz;
      }
  };

  struct StateHash {
      size_t operator()(const State& s) const {
          auto hash_combine = [](size_t lhs, size_t rhs) {
              return lhs ^ (rhs + 0x9e3779b9 + (lhs << 6) + (lhs >> 2));
          };

          size_t hash = std::hash<float>()(s.x);
          hash = hash_combine(hash, std::hash<float>()(s.y));
          hash = hash_combine(hash, std::hash<float>()(s.z));
          // hash = hash_combine(hash, std::hash<float>()(s.qw));
          // hash = hash_combine(hash, std::hash<float>()(s.qx));
          // hash = hash_combine(hash, std::hash<float>()(s.qy));
          // hash = hash_combine(hash, std::hash<float>()(s.qz));
          return hash;
      }
  };

  struct CompareCost {
      bool operator()(const std::pair<float, State>& a, const std::pair<float, State>& b) const {
          return a.first > b.first;
      }
  };

  virtual uint32_t makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost, std::string& message) override;
  virtual std::vector<std::pair<mesh_map::Vector, float>> getAdjacentPositions(const mesh_map::Vector& vertex,  int pointsPerEdge);
  virtual std::vector<State> findMinimalCostPath(const geometry_msgs::PoseStamped& original_start, const geometry_msgs::PoseStamped& original_goal, std::function<double(const State&, const State&)> kino_dynamic_cost_function);
  virtual nav_msgs::Path getNavPathFromStates(const std::vector<State> &path);
  virtual double calculateSteeringAngle(const std::vector<double>& current_state, const std::vector<double>& next_state, double L, double dt);
  virtual float getKinodynamicCost(const State& from, const State& to);
  virtual nav_msgs::Path getCvpPath(std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>>& path, const mesh_map::Vector& goal_vec, double& cost);
  virtual float calculateCostAtPosition(const mesh_map::Vector& position); 
  virtual std::pair<float, float> vectorFieldCost(); 
  virtual nav_msgs::Path getBsplinePath(const std::vector<State>& path);
  virtual Eigen::Vector3d getPosition(const lvr2::VertexHandle& vertex_handle);
  virtual void  savePathAndNormals(const std::vector<mesh_map::Vector>& path, const std::string& pathFileName, const std::string& normalsFileName);
  virtual State getStateAtPosition(const mesh_map::Vector position);
  virtual State getStateAtPose(const mesh_map::Vector position, const geometry_msgs::Pose pose);
  virtual double getHeadingFromState(const State& state);
  virtual State poseToState(const geometry_msgs::Pose& pose);
  virtual float evaluatePathFeasibility(const nav_msgs::Path& path);
  virtual float evaluateTransition(const State& from, const State& to);

  /**
   * @brief Dynamic reconfigure callback specific to KinodynamicWavefrontPlanner
   */
  void reconfigureCallback(kinodynamic_wavefront_planner::KinodynamicWavefrontPlannerConfig& cfg, uint32_t level);

private:
  KinodynamicWavefrontPlannerConfig kinodynamic_config;

};

}  // namespace kinodynamic_wavefront_planner

#endif  // MESH_NAVIGATION__KINODYNAMIC_WAVEFRONT_PLANNER_H
