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

#include <openmore/replanners/DRRT.h>

namespace openmore
{
DynamicRRT::DynamicRRT(Eigen::VectorXd& current_configuration, PathPtr& current_path, const double& max_time, const TreeSolverPtr& solver,
                       const TraceLoggerPtr& logger)
  : ReplannerBase(current_configuration, current_path, max_time, solver, logger)
{
  // Solver must be RRT
  const std::type_info& ti1 = typeid(RRT);
  const std::type_info& ti2 = typeid(*solver);

  RRTPtr tmp_solver;
  if (std::type_index(ti1) != std::type_index(ti2))
  {
    tmp_solver = std::make_shared<RRT>(solver->getMetrics(), solver->getChecker(), solver->getSampler(), logger_);
    tmp_solver->importFromSolver(solver);  // copy the required fields
  }
  else
  {
    tmp_solver = std::static_pointer_cast<RRT>(solver);
  }

  solver_ = tmp_solver;

  // Search the entire space when rebuilding the tree
  sampler_ = std::make_shared<UniformSampler>(lb_, ub_, logger_);
  solver_->setSampler(sampler_);
  tree_is_trimmed_ = false;
}

void DynamicRRT::fixTree(const NodePtr& node_replan, const NodePtr& root, std::vector<NodePtr>& old_nodes, std::vector<double>& old_connections_costs)
{
  if (success_)
    return;

  if (tree_is_trimmed_)  // rebuild the tree adding the old path (old_nodes and old_connections_costs) as a branch
  {
    std::reverse(old_nodes.begin(), old_nodes.end());  // to sort the nodes from goal to start
    std::reverse(old_connections_costs.begin(), old_connections_costs.end());

    assert(not trimmed_tree_->isInTree(node_replan));
    assert(old_nodes.front() == goal_node_);

    std::vector<ConnectionPtr> restore_old_path;
    for (unsigned int i = 1; i < old_nodes.size(); i++)  // the goal (i = 0) is in the tree (it is the root)
    {
      if (trimmed_tree_->isInTree(old_nodes.at(i)))
        continue;
      else
      {
        for (unsigned int j = i; j < old_nodes.size(); j++)
        {
          ConnectionPtr conn = std::make_shared<Connection>(old_nodes.at(j - 1), old_nodes.at(j), logger_);
          conn->setCost(old_connections_costs.at(j - 1));
          conn->add();

          restore_old_path.push_back(conn);
        }
        break;
      }
    }

    trimmed_tree_->addBranch(restore_old_path);
    tree_is_trimmed_ = false;

    trimmed_tree_->changeRoot(root);  // the old root (the starting point of the current path)
    current_path_->setConnections(trimmed_tree_->getConnectionToNode(goal_node_));

    assert([&]() -> bool {
      if (std::find(old_nodes.begin(), old_nodes.end(), node_replan) >= old_nodes.end())
      {
        if (trimmed_tree_->isInTree(node_replan))
          return false;
      }
      return true;
    }());
  }
  else
  {
    assert(trimmed_tree_->isInTree(node_replan));

    if (std::find(old_nodes.begin(), old_nodes.end(), node_replan) == old_nodes.end())  // if the node_replan has been added to the path and tree, remove it. If
                                                                                        // it was already present, do not remove it
    {
      assert(node_replan->getParentConnectionsSize() == 1 && node_replan->getChildConnectionsSize() == 1);

      NodePtr parent = node_replan->getParents().front();
      NodePtr child = node_replan->getChildren().front();

      ConnectionPtr conn = std::make_shared<Connection>(parent, child, logger_);
      double cost = node_replan->parentConnection(0)->getCost() + child->parentConnection(0)->getCost();
      conn->setCost(cost);
      conn->add();

      trimmed_tree_->removeNode(node_replan);
      assert(not trimmed_tree_->isInTree(node_replan));
    }
    trimmed_tree_->changeRoot(root);
    current_path_->setConnections(trimmed_tree_->getConnectionToNode(goal_node_));
  }

  assert(trimmed_tree_->getRoot() == root);
}

bool DynamicRRT::trimInvalidTree(NodePtr& node)
{
  auto tic = graph_time::now();

  bool trimmed = false;
  TreePtr tree = current_path_->getTree();

  NodePtr child;
  unsigned int removed_nodes = 0;   // will not be used;
  std::vector<NodePtr> white_list;  // will not be used;

  // Firstly trim the tree starting from the path to node
  bool obstructed;
  std::vector<ConnectionPtr> node2goal = tree->getConnectionToNode(node);  // Note: the root must be the goal (set in regrowRRT())
  for (const ConnectionPtr& conn : node2goal)
  {
    if (toSeconds(graph_time::now(), tic) >= max_time_)
    {
      if (verbose_)
        CNR_INFO(logger_, "Time to trim expired");

      break;
    }

    obstructed = false;
    if (not conn->isRecentlyChecked())
    {
      if (not checker_->checkConnection(conn))
      {
        conn->setCost(std::numeric_limits<double>::infinity());
        obstructed = true;
      }

      conn->setRecentlyChecked(true);
      checked_connections_.push_back(conn);
    }
    else
    {
      if (conn->getCost() == std::numeric_limits<double>::infinity())
        obstructed = true;
    }

    if (obstructed)
    {
      child = conn->getChild();
      tree->purgeFromHere(child, white_list,
                          removed_nodes);  // remove the successors and the connection from parent to child

      trimmed = true;
      break;
    }
  }

  if (not trimmed)
    return false;

  // Now check the remaining tree
  bool trimmed_again;
  do
  {
    trimmed_again = false;

    std::vector<NodePtr> leaves;
    std::vector<ConnectionPtr> path_connections;

    tree->getLeaves(leaves);
    for (const NodePtr& leave : leaves)
    {
      if (not tree->checkPathToNode(leave, checked_connections_, path_connections))
      {
        for (const ConnectionPtr& conn : path_connections)
        {
          if (conn->getCost() == std::numeric_limits<double>::infinity())
          {
            child = conn->getChild();
            tree->purgeFromHere(child, white_list, removed_nodes);

            trimmed = true;
            trimmed_again = true;

            break;
          }
        }
        if (trimmed_again)
          break;
      }
    }
  } while (trimmed_again);

  trimmed_tree_ = tree;
  return trimmed;
}

bool DynamicRRT::regrowRRT(NodePtr& node)
{
  auto tic = graph_time::now();

  // Set the goal as the root
  if (not current_path_->getTree()->changeRoot(goal_node_))  // revert the tree so the goal is the root
  {
    CNR_ERROR(logger_, "The goal can't be set as root!");
    CNR_INFO(logger_, "Goal node: " << goal_node_ << "\n" << *goal_node_);
    CNR_INFO(logger_, "Current path end node: " << current_path_->getGoalNode() << "\n" << *current_path_->getGoalNode());

    throw std::runtime_error("The goal can't be set as root!");
  }

  if (not tree_is_trimmed_)
  {
    // Trim the invalid parts of the tree
    if (not trimInvalidTree(node))
    {
      if (verbose_)
        CNR_INFO(logger_, "Tree not trimmed");

      tree_is_trimmed_ = false;
      return false;
    }
    else
      tree_is_trimmed_ = true;
  }

  // Regrow the tree
  double max_distance = trimmed_tree_->getMaximumDistance();
  assert(max_distance > 0.0);

  double time = toSeconds(graph_time::now(), tic);
  while (time < max_time_ && not success_)
  {
    NodePtr new_node;
    Eigen::VectorXd conf = sampler_->sample();
    if (trimmed_tree_->extend(conf, new_node))
    {
      assert(new_node->getParentConnectionsSize() == 1);
      new_node->getParentConnections().front()->setRecentlyChecked(true);
      checked_connections_.push_back(new_node->getParentConnections().front());  // this line is the reason why solver_
                                                                                 // is not in charge of re-build the
                                                                                 // tree here..

      if ((new_node->getConfiguration() - node->getConfiguration()).norm() < max_distance)
      {
        if (checker_->checkConnection(new_node->getConfiguration(), node->getConfiguration()))
        {
          assert(node->getParentConnectionsSize() == 0 && node->getChildConnectionsSize() == 0);

          ConnectionPtr conn = std::make_shared<Connection>(new_node, node, logger_);
          conn->setCost(metrics_->cost(new_node, node));
          conn->add();

          conn->setRecentlyChecked(true);
          checked_connections_.push_back(conn);

          trimmed_tree_->addNode(node);

          // Set the root in the node and extract the new path
          trimmed_tree_->changeRoot(node);
          replanned_path_ = std::make_shared<Path>(trimmed_tree_->getConnectionToNode(goal_node_), metrics_, checker_, logger_);
          replanned_path_->setTree(trimmed_tree_);

          solver_->setSolution(replanned_path_);  // set trimmed_tree_ as solver_'s tree
          tree_is_trimmed_ = false;

          success_ = true;
          break;
        }
      }
    }
    time = toSeconds(graph_time::now(), tic);
  }

  return success_;
}
bool DynamicRRT::replan()
{
  double cost_from_conf = current_path_->getCostFromConf(current_configuration_);
  return replan(cost_from_conf);
}

bool DynamicRRT::replan(const double& cost_from_conf)
{
  success_ = false;

  if (cost_from_conf == std::numeric_limits<double>::infinity())
  {
    if (verbose_)
      CNR_WARN(logger_, "Current path obstructed");

    std::vector<NodePtr> path_nodes = current_path_->getNodes();  // save nodes pointers (the same pointers stored in the tree)
    assert(path_nodes.front() == current_path_->getTree()->getRoot());

    std::vector<double> connections_costs;
    for (const ConnectionPtr& conn : current_path_->getConnections())
      connections_costs.push_back(conn->getCost());

    assert([&]() -> bool {
      for (const NodePtr& n : path_nodes)
      {
        if (not current_path_->getTree()->isInTree(n))
          return false;
      }
      return true;
    }());

    NodePtr root = current_path_->getTree()->getRoot();
    assert(root == current_path_->getConnections().front()->getParent());

    ConnectionPtr conn = current_path_->findConnection(current_configuration_);
    node_replan_ = current_path_->addNodeAtCurrentConfig(current_configuration_, conn, true, is_a_new_node_);

    if (verbose_)
      CNR_INFO(logger_, "Starting node for replanning: \n" << *node_replan_);

    checked_connections_.clear();
    checked_connections_ = current_path_->getSubpathFromNode(node_replan_)->getConnections();
    std::for_each(checked_connections_.begin(), checked_connections_.end(), [&](ConnectionPtr c) { c->setRecentlyChecked(true); });

    solver_->setSampler(sampler_);  // the uniform sampler

    if (not regrowRRT(node_replan_))  // root is goal
    {
      fixTree(node_replan_, root, path_nodes, connections_costs);
      replanned_path_ = current_path_;
      success_ = false;
    }
    else
      success_ = true;

    std::for_each(checked_connections_.begin(), checked_connections_.end(), [&](ConnectionPtr c) { c->setRecentlyChecked(false); });
  }
  else  // replan not needed
  {
    assert(current_path_->isValidFromConf(current_configuration_));

    success_ = false;
    replanned_path_ = current_path_;
  }

  return success_;
}
}  // namespace openmore
