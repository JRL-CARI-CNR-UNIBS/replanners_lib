﻿#include "replanners_lib/replanners/anytimeDRRT.h"

namespace openmore
{

AnytimeDynamicRRT::AnytimeDynamicRRT(Eigen::VectorXd& current_configuration,
                                     PathPtr& current_path,
                                     const double& max_time,
                                     const TreeSolverPtr &solver,
                                     const cnr_logger::TraceLoggerPtr& logger): DynamicRRT(current_configuration,current_path,max_time,solver,logger)
{
  const std::type_info& ti1 = typeid(AnytimeRRT);
  const std::type_info& ti2 = typeid(*solver);

  AnytimeRRTPtr tmp_solver;
  if(std::type_index(ti1) != std::type_index(ti2))
  {
    tmp_solver = std::make_shared<AnytimeRRT>(solver->getMetrics(), solver->getChecker(), solver->getSampler());
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
  ros::WallTime tic = ros::WallTime::now();

  bool success = false;

  AnytimeRRTPtr forced_cast_solver = std::static_pointer_cast<AnytimeRRT>(solver_);  //solver_ is of type AnytimeRRT (see constructor)

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
  forced_cast_solver->setPathCost(path_cost);

  int n_fail = 0;
  PathPtr solution;
  double cost2beat;
  double time = (ros::WallTime::now()-tic).toSec();
  while(time<max_time && n_fail<FAILED_ITER)
  {
    cost2beat = (1-imprv)*path_cost;

    if(verbose_)
      CNR_INFO(logger_,"Path cost: "<<path_cost<<", cost2beat: "<<cost2beat);

    NodePtr start_node = std::make_shared<Node>(node->getConfiguration());
    NodePtr goal_node  = std::make_shared<Node>(goal_node_->getConfiguration());

    bool improved = forced_cast_solver->improve(start_node,goal_node,solution,cost2beat,10000,(max_time-time));

    if(improved)
    {
      replanned_path_ = solution;
      goal_node_ = goal_node;
      success = true;
      n_fail = 0;

      assert(replanned_path_->getConnections().back()->getChild()->getConfiguration() == goal_node->getConfiguration());

      solver_->setStartTree(solution->getTree());
      solver_->setSolution(solution,true);

      if(verbose_)
        CNR_INFO(logger_,"Improved cost: "<<solution->cost());

      assert(replanned_path_->getTree());
    }
    else
    {
      n_fail +=1;

      if(verbose_)
        CNR_INFO(logger_,"Not improved");
    }

    time = (ros::WallTime::now()-tic).toSec();
  }

  return success;
}

bool AnytimeDynamicRRT::replan()
{
  ros::WallTime tic = ros::WallTime::now();

  success_ = false;
  double cost_from_conf = current_path_->getCostFromConf(current_configuration_);

  if(cost_from_conf == std::numeric_limits<double>::infinity())
  {
    if(DynamicRRT::replan(cost_from_conf))
    {
      success_ = true;

      current_path_ = replanned_path_;  //try to improve the path replanned with regrowRRT()

      solver_->setStartTree(current_path_->getTree());
      solver_->setSolution(current_path_,true);

      double max_time_impr = 0.98*max_time_-(ros::WallTime::now()-tic).toSec();
      if(improvePath(node_replan_,max_time_impr)) //if not improved, success_ = true anyway beacuse a new path has been found with regrowRRT()
      {
        solver_->setStartTree(replanned_path_->getTree());
        solver_->setSolution(replanned_path_,true);       //should be after setStartTree
      }
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

    solver_->setStartTree(current_path_->getTree());
    solver_->setSolution(current_path_,true);       //should be after setStartTree

    double max_time_impr = 0.98*max_time_-(ros::WallTime::now()-tic).toSec();
    if(improvePath(node_replan_,max_time_impr))
    {
      solver_->setStartTree(replanned_path_->getTree());
      solver_->setSolution(replanned_path_,true);   //should be after setStartTree

      success_ = true;
    }
    else
      success_ = false;
  }

  return success_;
}
}
