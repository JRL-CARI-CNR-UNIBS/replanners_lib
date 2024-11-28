/*
Copyright (c) 2024

JRL-CARI CNR-STIIMA/UNIBS
Cesare Tonola, c.tonola001@unibs.it
Manuel Beschi, manuel.beschi@unibs.it

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <openmore/replanners/replanner_base.h>
#include <graph_core/solvers/rrt.h>
#include <future>
#include <mutex>

/**
 * @file MPRRT.h
 * @brief From the paper High-frequency replanning under uncertainty using parallel sampling-based motion planning (https://ieeexplore.ieee.org/document/7027233).
 */

namespace openmore
{

/**
 * @class MPRRT
 * @brief Class for high-frequency replanning under uncertainty using parallel sampling-based motion planning.
 *
 * This algorithm runs multiple parallel RRT instances concurrently to reach the same goal,
 * increasing the chances of finding a valid path within the given time constraints. With the number of
 * processors going to infinity, the probability of finding the optimal solution approaches one.
 */
class MPRRT;
typedef std::shared_ptr<MPRRT> MPRRTPtr;

class MPRRT: public ReplannerBase
{
protected:

  /**
   * @brief Number of parallel plannings to be executed.
   */
  unsigned int number_of_parallel_plannings_;

  /**
   * @brief Vector of RRT solvers used for parallel planning.
   */
  std::vector<RRTPtr> solver_vector_;

  /**
   * @brief Vector of the best paths foudn by each parallel RRT.
   */
  std::vector<PathPtr> connecting_path_vector_;

  /**
   * @brief Replace the first and last nodes of the new path found with the replanning node and goal node (same configuration but different pointers).
   *
   * @param connecting_path_conn Connections forming the new path to the goal.
   * @param path1_node replanning node.
   * @return A shared pointer to the new path.
   */
  PathPtr concatWithNewPathToGoal(const std::vector<ConnectionPtr>& connecting_path_conn, const NodePtr& path1_node);

  /**
   * @brief Computes a RRT asynchronously. This is the function executed by each parallel thread.
   *
   * @param path1_node_conf Configuration of the start node.
   * @param path2_node_conf Configuration of the goal node.
   * @param current_solution_cost Cost of the current solution, which should be improved.
   * @param index Index of the solver in the solver vector.
   * @return True if a connecting path was successfully computed, false otherwise.
   */
  bool asyncComputeConnectingPath(const Eigen::VectorXd path1_node_conf, const Eigen::VectorXd path2_node_conf, const double current_solution_cost, const int index);

  /**
   * @brief Computes a path between two nodes. This function is called by asyncComputeConnectingPath.
   *
   * @param path1_node_fake Start node.
   * @param path2_node_fake Goal node.
   * @param current_solution_cost Cost of the current solution.
   * @param max_time Maximum allowed time for computation.
   * @param connecting_path Output parameter for the computed connecting path.
   * @param directly_connected Output parameter indicating if the nodes were directly connected (one straight connection).
   * @param solver Solver used for computing the path.
   * @return True if a connecting path was successfully computed, false otherwise.
   */
  bool computeConnectingPath(const NodePtr &path1_node_fake, const NodePtr &path2_node_fake, const double &current_solution_cost, const double max_time, PathPtr &connecting_path, bool &directly_connected, TreeSolverPtr &solver);

  /**
   * @brief The function which orchestrates the replannign algorithm by calling the RRT execution (asyncComputeConnectingPath) on multiple threads.
   *
   * @param node Replanning node to start the connection attempt from.
   * @return True if the node was successfully connected to the goal, false otherwise.
   */
  bool connect2goal(const NodePtr& node);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor for the MPRRT class.
   *
   * @param current_configuration Current configuration of the robot.
   * @param current_path Current path of the robot.
   * @param max_time Maximum time allowed for replanning.
   * @param solver Solver for the tree. If it is not RRT, it is converted into RRT.
   * @param logger Logger for trace information.
   * @param number_of_parallel_plannings Number of parallel plannings to be executed. Default is 1.
   */
  MPRRT(Eigen::VectorXd& current_configuration,
        PathPtr& current_path,
        const double& max_time,
        const TreeSolverPtr& solver,
        const TraceLoggerPtr &logger,
        const unsigned int& number_of_parallel_plannings = 1);

  /**
   * @brief Execute the replanning algorithm.
   * @return True if replanning was successful, false otherwise.
   */
  bool replan() override;
};
}
