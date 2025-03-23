#ifndef MESH_NAVIGATION__GAKD_MESH_PLANNER_H
#define MESH_NAVIGATION__GAKD_MESH_PLANNER_H

#include "wave_front_planner/wave_front_planner.h"
#include <gakd_mesh_planner/GakdMeshPlannerConfig.h>

namespace gakd_mesh_planner
{
  typedef std::vector<double> State;
  typedef std::vector<State> Trajectory;
  typedef std::vector<double> Control;
  typedef std::vector<Control> ControlSequence;
class GakdMeshPlanner : public wave_front_planner::WaveFrontPlanner
{
public:
  typedef boost::shared_ptr<GakdMeshPlanner> Ptr;

  /**
   * @brief Constructor
   */
  GakdMeshPlanner();

  /**
   * @brief Destructor
   */
  virtual ~GakdMeshPlanner();

  

  virtual uint32_t makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost, std::string& message) override;
  virtual boost::optional<mesh_map::Vector>  calculateDirectionAtPosition(const mesh_map::Vector& position);
 
  
  /**
   * @brief Dynamic reconfigure callback specific to GakdMeshPlanner
   */
  void reconfigureCallback(gakd_mesh_planner::GakdMeshPlannerConfig& cfg, uint32_t level);


  Trajectory ga_planner(const State& start_point, const State& goal_point, double time_horizon);
  State dynamics(const State& x, const Control& u, double dt);
  double cost_function(const State& current_state, const State& next_state, 
                                        const Control& control, const State& state_target);
  Control random_control();
  vector<ControlSequence> generate_population(int size, int horizon);
  std::pair<ControlSequence, ControlSequence> select_parents(const vector<ControlSequence>& population, 
                                                                        const vector<double>& fitnesses);
  std::pair<ControlSequence, ControlSequence> crossover(const ControlSequence& parent1, 
                                                                    const ControlSequence& parent2);
  ControlSequence mutate(ControlSequence control_sequence, double mutation_rate );
  ControlSequence genetic_algorithm(const State& state_init, const State& state_target,
                                                    double dt,  int pop_size , int generations ,
                                                    int time_horizon, double mutation_rate);

  float computeDirectionCost(const mesh_map::Vector& current, const mesh_map::Vector& next);
  boost::optional<std::pair<std::vector<mesh_map::Vector>, mesh_map::Vector>> findClosestFace(const mesh_map::Vector &position);
  boost::optional<std::pair<mesh_map::Vector, float>> projectToFaceAndDistance(const mesh_map::Vector& position);

  nav_msgs::Path getCvpPath(std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>>& path, const mesh_map::Vector& goal_vec, double& cost);
private:
  GakdMeshPlannerConfig kinodynamic_config;

};

}  // namespace gakd_mesh_planner

#endif  // MESH_NAVIGATION__GAKD_MESH_PLANNER_H
