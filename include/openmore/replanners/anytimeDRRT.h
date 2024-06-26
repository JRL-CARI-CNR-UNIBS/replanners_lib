#pragma once

#include <openmore/replanners/DRRT.h>
#include <graph_core/solvers/anytime_rrt.h>

/**
 * @file anytimeDRRT.h
 * @brief From the paper Anytime, Dynamic Planning in High-dimensional Search Spaces (https://ieeexplore.ieee.org/document/4209270)
 */

namespace openmore
{

/**
 * @class AnytimeDynamicRRT
 * @brief Class for dynamic replanning using Anytime RRT in high-dimensional search spaces.
 *
 * This class initially utilizes Dynamic RRT (DRRT) to find a valid solution. Once an initial path is established,
 * it leverages Anytime RRT to iteratively improve the path within the remaining available time.
 */
class AnytimeDynamicRRT;
typedef std::shared_ptr<AnytimeDynamicRRT> AnytimeDynamicRRTPtr;

class AnytimeDynamicRRT: public DynamicRRT
{

#define FAILED_ITER 3 //maximum number of failed path improvement attempts with AnytimeRRT

protected:

  /**
   * @brief Attempts to improve the replanned path using AnytimeRRT.
   *
   * This function tries to enhance the path quality by finding a better path
   * from the given node to the goal within the provided maximum time using AnytimeRRT.
   *
   * @param node Node to start improving from.
   * @param max_time Maximum time allowed for improving the path.
   * @return True if the path was successfully improved, false otherwise.
   */
  bool improvePath(NodePtr &node, const double& max_time);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor for the AnytimeDynamicRRT class.
   *
   * @param current_configuration Current configuration of the robot.
   * @param current_path Current path of the robot.
   * @param max_time Maximum time allowed for replanning.
   * @param solver Solver for the tree. If it is not AnytimeRRT, it is converted into AnytimeRRT.
   * @param logger Logger for trace information.
   */
  AnytimeDynamicRRT(Eigen::VectorXd& current_configuration,
                    PathPtr& current_path, const double& max_time,
                    const TreeSolverPtr &solver, const cnr_logger::TraceLoggerPtr &logger);

  /**
   * @brief Execute the replanning algorithm with anytime improvements.
   *
   * This function attempts to replan the path from the current configuration using DRRT.
   * If the initial replanning succeeds, it further tries to improve the path within the allowed time using AnytimeRRT.
   *
   * @return True if replanning was successful, false otherwise.
   */
  bool replan() override;
};
}

