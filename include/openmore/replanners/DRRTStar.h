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
#include <graph_core/samplers/ball_sampler.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/graph/subtree.h>

/**
 * @file DRRTStar.h
 * @brief From the paper Dynamic path planning and replanning for mobile robots using RRT*
 * (https://ieeexplore.ieee.org/document/8122814).
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

class DynamicRRTStar : public ReplannerBase
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
  bool connectBehindObs(const NodePtr& node);

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
  DynamicRRTStar(Eigen::VectorXd& current_configuration, PathPtr& current_path, const double& max_time,
                 const TreeSolverPtr& solver, const TraceLoggerPtr& logger);

  /**
   * @brief Executes the replanning algorithm.
   *
   * @return True if replanning was successful, false otherwise.
   */
  bool replan() override;
};
}  // namespace openmore
