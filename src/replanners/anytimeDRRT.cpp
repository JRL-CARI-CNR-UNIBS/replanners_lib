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

#include <openmore/replanners/anytimeDRRT.h>

namespace openmore
{

AnytimeDynamicRRT::AnytimeDynamicRRT(Eigen::VectorXd& current_configuration,
                                     PathPtr& current_path,
                                     const double& max_time,
                                     const TreeSolverPtr &solver,
                                     const TraceLoggerPtr& logger): DynamicRRT(current_configuration,current_path,max_time,solver,logger)
{
  const std::type_info& ti1 = typeid(AnytimeRRT);
  const std::type_info& ti2 = typeid(*solver);

  AnytimeRRTPtr tmp_solver;
  if(std::type_index(ti1) != std::type_index(ti2))
  {
    tmp_solver = std::make_shared<AnytimeRRT>(solver->getMetrics(), solver->getChecker(), solver->getSampler(), logger_);
    tmp_solver->importFromSolver(solver); //copy the required fields
  }
  else
    tmp_solver = std::static_pointer_cast<AnytimeRRT>(solver);

  solver_ = tmp_solver;

  assert(tmp_solver->solved());
  assert(solver_->getSolution());
  assert(solver_->getStartTree());
}

bool AnytimeDynamicRRT::improvePath(NodePtr &node, const double& max_time)
{
  auto tic = graph_time::now();

  bool success = false;

  AnytimeRRTPtr forced_cast_solver = std::static_pointer_cast<AnytimeRRT>(solver_); //solver_ is of type AnytimeRRT (see constructor)

  if(replanned_path_)
  {
    if(current_path_->getTree() != replanned_path_->getTree())
    {
      CNR_INFO(logger_,"Current path tree, replanned path tree: "<<current_path_->getTree()<<" "<<replanned_path_->getTree());
      assert(0);
    }
  }

  if(current_path_->getTree() != solver_->getStartTree())
  {
    CNR_INFO(logger_,"Current path tree, solver tree: "<<current_path_->getTree()<<" "<<solver_->getStartTree());
    assert(0);
  }

  assert(forced_cast_solver->getStartTree());
  assert(forced_cast_solver->getSolution());

  double imprv = forced_cast_solver->getCostImpr();
  double path_cost = solver_->getSolution()->getCostFromConf(node->getConfiguration());

  InformedSamplerPtr informed_sampler = std::make_shared<InformedSampler>(node->getConfiguration(),goal_node_->getConfiguration(),lb_,ub_,logger_,path_cost);
  forced_cast_solver->setSampler(informed_sampler);

  //  forced_cast_solver->setPathCost(path_cost); //CHECK!!!

  int n_fail = 0;
  PathPtr solution;
  double cost2beat;
  double time = toSeconds(graph_time::now(),tic);
  while(time<max_time && n_fail<FAILED_ITER)
  {
    cost2beat = (1-imprv)*path_cost;

    if(verbose_)
      CNR_INFO(logger_,"Path cost: "<<path_cost<<", cost2beat: "<<cost2beat);

    NodePtr start_node = std::make_shared<Node>(node->getConfiguration(),logger_);
    NodePtr goal_node  = std::make_shared<Node>(goal_node_->getConfiguration(),logger_);

    bool improved = forced_cast_solver->improve(start_node,goal_node,solution,cost2beat,10000,(max_time-time));

    if(improved)
    {
      replanned_path_ = solution;
      goal_node_ = goal_node;
      success = true;
      n_fail = 0;

      assert(replanned_path_->getConnections().back()->getChild()->getConfiguration() == goal_node->getConfiguration());

      solver_->setSolution(solution); //set solution's tree as solver's tree
      path_cost = solution->cost(); //CHECK

      if(verbose_)
        CNR_INFO(logger_,"Improved cost: "<<path_cost);

      assert(replanned_path_->getTree());
    }
    else
    {
      n_fail +=1;

      if(verbose_)
        CNR_INFO(logger_,"Not improved");
    }

    time = toSeconds(graph_time::now(),tic);
  }

  return success;
}

bool AnytimeDynamicRRT::replan()
{
  auto tic = graph_time::now();

  success_ = false;
  double cost_from_conf = current_path_->getCostFromConf(current_configuration_);

  if(cost_from_conf == std::numeric_limits<double>::infinity())
  {
    if(DynamicRRT::replan(cost_from_conf))
    {
      success_ = true;

      current_path_ = replanned_path_;  //try to improve the path replanned with regrowRRT()
      solver_->setSolution(current_path_);

      double max_time_impr = 0.98*max_time_-toSeconds(graph_time::now(),tic);
      if(improvePath(node_replan_,max_time_impr)) //if not improved, success_ = true anyway beacuse a new path has been found with regrowRRT()
        solver_->setSolution(replanned_path_);
    }
    else
    {
      success_ = false;
      if(verbose_)
        CNR_ERROR(logger_,"Tree can not be regrown using regrowRRT");
    }
  }
  else
  {
    ConnectionPtr conn = current_path_->findConnection(current_configuration_);
    node_replan_ = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,false);

    solver_->setSolution(current_path_);

    double max_time_impr = 0.98*max_time_-toSeconds(graph_time::now(),tic);
    if(improvePath(node_replan_,max_time_impr))
    {
      solver_->setSolution(replanned_path_);
      success_ = true;
    }
    else
      success_ = false;
  }

  return success_;
}
}
