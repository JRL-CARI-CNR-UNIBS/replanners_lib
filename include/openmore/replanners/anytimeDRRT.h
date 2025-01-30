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

#include <openmore/replanners/DRRT.h>
#include <graph_core/solvers/anytime_rrt.h>

/**
 * @file anytimeDRRT.h
 * @brief From the paper Anytime, Dynamic Planning in High-dimensional Search Spaces
 * (https://ieeexplore.ieee.org/document/4209270)
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

class AnytimeDynamicRRT : public DynamicRRT
{
#define FAILED_ITER 3  // maximum number of failed path improvement attempts with AnytimeRRT

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
  bool improvePath(NodePtr& node, const double& max_time);

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
  AnytimeDynamicRRT(Eigen::VectorXd& current_configuration, PathPtr& current_path, const double& max_time, const TreeSolverPtr& solver,
                    const TraceLoggerPtr& logger);

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
}  // namespace openmore
