#pragma once

#include <openmore/replanners/replanner_base.h>
#include <graph_core/samplers/ball_sampler.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/graph/subtree.h>

/**
 * @file DRRTStar.h
 * @brief From the paper Dynamic path planning and replanning for mobile robots using RRT* (https://ieeexplore.ieee.org/document/8122814).
 */

namespace openmore
{

/**
 * @class DynamicRRTStar
 * @brief Class for dynamic path planning and replanning for mobile robots using RRT*.
 *
 * This algorithm rewires the tree in the vicinity of obstacles that invalidate the current path,
 * in order to find a new valid solution to reach the goal.
 */
class DynamicRRTStar;
typedef std::shared_ptr<DynamicRRTStar> DynamicRRTStarPtr;

class DynamicRRTStar: public ReplannerBase
{
protected:

  /**
   * @brief Finds the last valid node before an obstacle on a given subpath.
   *
   * @param subpath The subpath to search within.
   * @param node_before The node before the obstacle.
   * @return True if a node before an obstacle is found, false otherwise.
   */
  bool nodeBeforeObs(const PathPtr& subpath, NodePtr& node_before);

  /**
   * @brief Finds the first valid node behind an obstacle on the current path.
   *
   * @param node_behind The node behind the obstacle.
   * @return True if a node behind an obstacle is found, false otherwise.
   */
  bool nodeBehindObs(NodePtr& node_behind);

  /**
   * @brief Connects the given node to the goal node behind an obstacle on the current path.
   *
   * @param node The node from which replanning starts.
   * @return True if the connection to the goal node is successful, false otherwise.
   */
  bool connectBehindObs(const NodePtr &node);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor for the DynamicRRTStar class.
   *
   * @param current_configuration Current configuration of the robot.
   * @param current_path Current path of the robot.
   * @param max_time Maximum time allowed for replanning.
   * @param solver Solver for the tree. If it is not RRT*, it is converted into RRT*.
   * @param logger Logger for trace information.
   */
  DynamicRRTStar(Eigen::VectorXd& current_configuration,
                 PathPtr& current_path,
                 const double& max_time,
                 const TreeSolverPtr &solver,
                 const cnr_logger::TraceLoggerPtr &logger);

  /**
    * @brief Executes the replanning algorithm.
    *
    * @return True if replanning was successful, false otherwise.
    */
  bool replan() override;
};
}
