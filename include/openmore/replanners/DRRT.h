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

#include <graph_core/samplers/uniform_sampler.h>
#include <openmore/replanners/replanner_base.h>
#include <graph_core/solvers/rrt.h>
#include <typeinfo>

/**
 * @file DRRT.h
 * @brief From the paper Replanning with RRTs (https://ieeexplore.ieee.org/document/1641879)
 */

namespace openmore
{

/**
 * @class DynamicRRT
 * @brief Class for dynamic replanning using Rapidly-exploring Random Trees (RRT).
 * The algorithm replans from the goal to a given node by trimming the invalid part of the tree and then restarting its growth to find a new valid solution.
 */
class DynamicRRT;
typedef std::shared_ptr<DynamicRRT> DynamicRRTPtr;

class DynamicRRT: public ReplannerBase
{
protected:
  /**
   * @brief Node to start replanning from.
   */
  NodePtr node_replan_;

  /**
   * @brief DRRT trims the tree to remove the invalid connections. trimmed_tree_ points to this trimmed tree.
   */
  TreePtr trimmed_tree_;

  /**
   * @brief Flag indicating if the tree has been trimmed.
   *
   * If true, the tree has been trimmed and must be rebuilt.
   * This is useful for saving the state of the tree for later iterations
   * if the tree's growth process has run out of time.
   */
  bool tree_is_trimmed_;

  /**
   * @brief Sampler for uniformly sampling during the tree's growth process.
   */
  UniformSamplerPtr sampler_;

  /**
   * @brief List of checked connections.
   *
   * At the beginning of the algorithm, all the tree's connections are validated.
   * To avoid multiple validations, each connection's `recently_checked` flag is
   * set to true, and the connection is then inserted into this vector. This
   * ensures that the flag can be reset later on at the end of the algorithm.
   */
  std::vector<ConnectionPtr> checked_connections_;

  /**
   * @brief Regrows the RRT tree to find a new collision-free path from the given node to the goal.
   *
   * This function is the core of the replanning algorithm and moves the tree's root to the goal,
   * trims the invalid part of the tree, and then rebuilds it to reach the given node with a new,
   * collision-free path, exploiting the remaining valid part of the trimmed tree.
   *
   * @param node Replanning node. The replanned path will start from this node and end at the goal.
   * @return True if the RRT was successfully regrown, false otherwise.
   */
  virtual bool regrowRRT(NodePtr& node);

  /**
   * @brief Trims the invalid parts of the current_path_'s tree.
   *
   * This function starts trimming the path from the goal to the node given as input.
   * This approach helps reduce calculations by immediately discarding a large part of the tree if the path is trimmed.
   * It assumes the tree's root is placed at the goal node. If the tree is trimmed, it is assigned to the member `trimmed_tree_`.
   *
   * @param node Node to start trimming from.
   * @return True if the tree was trimmed, false otherwise.
   */
  virtual bool trimInvalidTree(NodePtr& node);

  /**
   * @brief Fixes the tree by adding the old path as a branch.
   *
   * If the previous iteration has trimmed the tree but not rebuilt it (e.g., due to lack of time),
   * this function "restores" a tree made only of the current path.
   *
   * @param node_replan Node to start replanning from.
   * @param root Root of the tree.
   * @param old_nodes List of old nodes (the path's nodes).
   * @param old_connections_costs List of old connection costs (the path's connections).
   */
  void fixTree(const NodePtr& node_replan, const NodePtr& root, std::vector<NodePtr> &old_nodes, std::vector<double> &old_connections_costs);

  /**
   * @brief Replans the path from the current configuration.
   * @param cost_from_conf CPath cost from the current configuration.
   * @return True if replanning was successful, false otherwise.
   */
  bool replan(const double& cost_from_conf);
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor for the DynamicRRT class.
   * @param current_configuration Current configuration of the robot.
   * @param current_path Current path of the robot.
   * @param max_time Maximum time allowed for replanning.
   * @param solver Solver for the tree. If it is not RRT, it is converted into RRT.
   * @param logger Logger for trace information.
   */
  DynamicRRT(Eigen::VectorXd& current_configuration,
             PathPtr& current_path,
             const double& max_time,
             const TreeSolverPtr &solver,
             const TraceLoggerPtr &logger);

  /**
   * @brief Gets the status of whether the tree is trimmed.
   * @return True if the tree is trimmed, false otherwise.
   */
  bool getTreeIsTrimmed()
  {
    return tree_is_trimmed_;
  }

  /**
   * @brief Execute the replanning algorithm.
   * @return True if replanning was successful, false otherwise.
   */
  virtual bool replan() override;
};
}
