#pragma once

#include <openmore/replanners/util/utils.h>

namespace openmore
{
/**
 * @class ReplannerBase
 * @brief Abstract base class representing a sampling-based (RRT-like) path replanner.
 */
class ReplannerBase;
typedef std::shared_ptr<ReplannerBase> ReplannerBasePtr;

class ReplannerBase: public std::enable_shared_from_this<ReplannerBase>
{
protected:
  /**
   * @brief The current robot configuration. Replanning starts at this or a subsequent configuration along the path.
   */
  Eigen::VectorXd current_configuration_;

  /**
   * @brief The current robot's path, which may be invalid due to newly spawned obstacles.
   * In such cases, one or more of its connections will have a cost equal to infinity.
   */
  PathPtr current_path_;

  /**
   * @brief The path replanned by the algorithm.
   * The replanner modifies the current_path_ (or its tree) and assigns the new collision-free path to the replanned_path_.
   */
  PathPtr replanned_path_;

  /**
   * @brief Solver for tree-based path planning. The replanner should use this solver to compute paths if needed.
   */
  TreeSolverPtr solver_;

  /**
   * @brief Metrics to evaluate the costs of paths and connections. For example, Euclidean metrics evaluate connections/paths length.
   * Note that the metrics do not detect obstacles; the collision checker does. Thus, check if a connection is collision-free.
   * If it is, compute its cost using the metrics; otherwise, set its cost to std::numeric_limits<double>::infinity().
   */
  MetricsPtr metrics_;

  /**
   * @brief Collision checker. It receives connections or paths as inputs and returns true or false based on their validity.
   */
  CollisionCheckerPtr checker_;

  /**
   * @brief Lower bounds of the configuration space.
   */
  Eigen::VectorXd lb_;

  /**
   * @brief Upper bounds of the configuration space.
   */
  Eigen::VectorXd ub_;

  /**
   * @brief The goal node to reach, which is set to the goal node of the current_path_ by default.
   */
  NodePtr goal_node_;

  /**
   * @brief Flag to indicate if a new node corresponding to the current_configuration_ has been created at the beginning of the algorithm.
   *
   * Replanning algorithms usually add a node corresponding to the current robot configuration at the beginning of the algorithm.
   * If a new node is created, this flag is set to true. This can be helpful if, for instance, you want to remove previously created nodes in subsequent iterations.
   */
  bool is_a_new_node_;

  /**
   * @brief Flag to indicate if replanning was successful.
   */
  bool success_;

  /**
   * @brief Flag to control the verbosity of logging.
   */
  bool verbose_;

  /**
   * @brief Maximum time allowed for replanning (e.g., 100 milliseconds). If a solution is not found within this time frame, the algorithm exits.
   */
  double max_time_;

  /**
   * @brief Logger for tracing.
   */
  TraceLoggerPtr logger_;

#ifdef GRAPH_DISPLAY_AVAILABLE
  /**
   * @brief Display object for representing nodes, connections, paths, etc., on Rviz when
   * ROS and the package graph_display are available (see CMakeLists.txt for more info about GRAPH_DISPLAY_AVAILABLE).
   * It is used for debugging purposes.
   */
  DisplayPtr disp_;
#endif

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor for ReplannerBase.
   * @param current_configuration Current configuration of the robot. Replannig will start from this or a subsequent configuration along the path.
   * @param current_path The current path traversed by the robot.
   * @param max_time Maximum time allowed for replanning.
   * @param solver Solver for tree-based path planning, e.g. RRT.
   * @param logger Logger for tracing.
   */
  ReplannerBase(const Eigen::VectorXd &current_configuration,
                const PathPtr &current_path,
                const double &max_time,
                const TreeSolverPtr& solver,
                const TraceLoggerPtr &logger);
  /**
   * @brief Destructor for ReplannerBase.
   */
  ~ReplannerBase();

  /**
   * @brief Get the replanned path.
   * @return The replanned path.
   */
  PathPtr getReplannedPath() const
  {
    return replanned_path_;
  }

  /**
   * @brief Get the current path.
   * @return The current path.
   */
  PathPtr getCurrentPath() const
  {
    return current_path_;
  }

  /**
   * @brief Set the current path. The success flag is set to false and the goal_node_ equal to the final node of the path.
   * @param path The new current path.
   */
  virtual void setCurrentPath(const PathPtr& path)
  {
    success_ = false;
    current_path_ = path;
    goal_node_  = current_path_->getConnections().back()->getChild();
  }

  /**
    * @brief Set the maximum time for replanning.
    * @param max_time The maximum time allowed for replanning.
    */
  void setMaxTime(const double& max_time)
  {
    max_time_ = max_time;
  }

  /**
   * @brief Set the verbosity for logging.
   * @param verbose Boolean flag to set verbosity.
   */
  virtual void setVerbosity(const bool& verbose)
  {
    verbose_ = verbose;
  }

  /**
   * @brief Set the current configuration of the robot. Replannig will start from this or a subsequent configuration along the path.
   * @param q The new current configuration.
   */
  virtual void setCurrentConf(const Eigen::VectorXd& q)
  {
    current_configuration_ = q;
    success_ = false;
  }

  /**
   * @brief Get the goal node.
   * @return The goal node.
   */
  NodePtr getGoal() const
  {
    return goal_node_;
  }

  /**
   * @brief Get the current configuration of the robot.
   * @return The current configuration.
   */
  Eigen::VectorXd getCurrentConf() const
  {
    return current_configuration_;
  }

  /**
   * @brief Get the solver used for tree-based path planning.
   * @return The solver.
   */
  TreeSolverPtr getSolver() const
  {
    return solver_;
  }

  /**
   * @brief Set the collision checker used by the algorithm, the solver, the current_path_ and replanned_path_.
   * @param checker The collision checker.
   */
  virtual void setChecker(const CollisionCheckerPtr &checker)
  {
    checker_ = checker;
    solver_->setChecker(checker);
    current_path_->setChecker(checker);

    if(replanned_path_)
      replanned_path_->setChecker(checker);
  }

#ifdef GRAPH_DISPLAY_AVAILABLE
  /**
   * @brief Set the Display object for visually representing nodes, connections, paths, etc., on Rviz when
   * ROS and the package graph_display are available (see CMakeLists.txt for more info about GRAPH_DISPLAY_AVAILABLE).
   * It is used for debugging purposes.
   * @param disp The display object.
   */
  void setDisp(const DisplayPtr &disp)
  {
    disp_ = disp;
  }
#endif

#ifdef GRAPH_DISPLAY_AVAILABLE
  /**
   * @brief Get the Display object used for visually representing nodes, connections, paths, etc., on Rviz when
   * ROS and the package graph_display are available (see CMakeLists.txt for more info about GRAPH_DISPLAY_AVAILABLE).
   * @return The display object.
   */
  DisplayPtr getDisp() const
  {
    return disp_;
  }
#endif

  /**
   * @brief Return if a node has been created corresponding to the replanning configuration.
   * @return True if the replanned nodehas been created, false otherwise.
   */
  bool replanNodeIsANewNode() const
  {
    return is_a_new_node_;
  }

  /**
   * @brief Get the success status of replanning.
   * @return True if replanning was successful, false otherwise.
   */
  bool getSuccess() const
  {
    return success_;
  }

  /**
   * @brief Pure virtual function to execute the replanning algorithm.
   * @return True if replanning was successful, false otherwise.
   */
  virtual bool replan() = 0;
};
}
