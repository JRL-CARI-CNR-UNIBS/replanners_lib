#include <openmore/replanners/MPRRT.h>

namespace openmore
{

MPRRT::MPRRT(Eigen::VectorXd& current_configuration,
             PathPtr& current_path,
             const double& max_time,
             const TreeSolverPtr& solver,
             const TraceLoggerPtr& logger,
             const unsigned int& number_of_parallel_plannings): ReplannerBase(current_configuration,current_path,max_time,solver,logger)
{
  // Solver should be RRT
  const std::type_info& ti1 = typeid(RRT);
  const std::type_info& ti2 = typeid(*solver);

  RRTPtr tmp_solver;

  if(std::type_index(ti1) != std::type_index(ti2))
  {
    tmp_solver = std::make_shared<RRT>(solver->getMetrics(), solver->getChecker(), solver->getSampler(), logger);
    tmp_solver->importFromSolver(solver); //copy the required fields
  }
  else
  {
    tmp_solver = std::static_pointer_cast<RRT>(solver);
  }

  solver_ = tmp_solver;

  if(number_of_parallel_plannings<1)
    number_of_parallel_plannings_ = 1;
  else
    number_of_parallel_plannings_ =  number_of_parallel_plannings;

  solver_vector_.clear();
  for(unsigned int i=0;i<number_of_parallel_plannings_;i++)
  {
    RRTPtr sv = std::make_shared<RRT>(metrics_->clone(),checker_->clone(),solver_->getSampler(),logger_); //solver will be overwritten
    sv->importFromSolver(solver_);

    solver_vector_.push_back(sv);
  }

  connecting_path_vector_.resize(number_of_parallel_plannings_,nullptr);
}

bool MPRRT::asyncComputeConnectingPath(const Eigen::VectorXd path1_node_conf,
                                       const Eigen::VectorXd path2_node_conf,
                                       const double current_solution_cost,
                                       const int index)
{
  auto tic = graph_time::now();

  TreeSolverPtr solver = solver_vector_.at(index);

  double best_cost = std::numeric_limits<double>::infinity();
  PathPtr best_solution = nullptr;
  bool success = false;
  int iter = 0;
  double time;

  // Compute RRT from replan configuration to goal configuration and save the best solution for this thread
  do
  {
    iter++;

    NodePtr path1_node = std::make_shared<Node>(path1_node_conf);
    NodePtr path2_node = std::make_shared<Node>(path2_node_conf);

    PathPtr connecting_path = nullptr;
    bool directly_connected = false;

    time = max_time_- graph_duration(graph_time::now()-tic).count();

    bool solved = computeConnectingPath(path1_node,path2_node,current_solution_cost,time,
                                        connecting_path,directly_connected,solver);

    if(solved)
    {
      success = true;

      double new_cost = connecting_path->cost();
      if(new_cost<best_cost)
      {
        best_solution = connecting_path;
        best_cost = new_cost;
      }
    }
  } while((0.98*max_time_-graph_duration(graph_time::now()-tic).count())>0.0);

  connecting_path_vector_.at(index) = best_solution;

  double cost;
  success?
        (cost = best_cost):
        (cost = current_solution_cost);

  if(verbose_)
    CNR_INFO(logger_,"\n--- THREAD REASUME ---\nthread n: "<<index<<"\nsuccess: "<<success<<"\nconnecting path cost: "<< cost<<"\nn iter: "<<iter<<" time: "<<graph_duration(graph_time::now()-tic).count());

  return success;
}

bool MPRRT::connect2goal(const NodePtr& node)
{
  success_ = false;
  bool solved = false;
  std::vector<std::shared_future<bool>> futures;

  double current_cost = current_path_->getCostFromConf(node->getConfiguration());

  if(current_cost <= 1.05*(node->getConfiguration()-goal_node_->getConfiguration()).norm())
  {
    success_ = false;
    return success_;
  }

  if(verbose_)
  {
    if(current_cost == std::numeric_limits<double>::infinity())
      CNR_WARN(logger_,"Current path obstructed");
  }

  // Launch multiple parallel RRTs
  for(unsigned int i=0; i<number_of_parallel_plannings_;i++)
  {
    int index = i;
    futures.push_back(std::async(std::launch::async,
                                 &MPRRT::asyncComputeConnectingPath,
                                 this,node->getConfiguration(),
                                 goal_node_->getConfiguration(),current_cost,index));
  }

  std::vector<double> marker_color;
  marker_color = {1.0,1.0,0.0,1.0};

  unsigned int idx_best_sol = -1;
  double best_cost = std::numeric_limits<double>::infinity();

  // Retrieve the best solution found
  for(unsigned int i=0; i<number_of_parallel_plannings_;i++)
  {
    if(futures.at(i).get())
    {
      assert(connecting_path_vector_.at(i));

      solved = true;
      double i_cost = connecting_path_vector_.at(i)->cost();

      if(i_cost<best_cost)
      {
        best_cost = i_cost;
        idx_best_sol = i;

        if(verbose_)
          CNR_INFO(logger_,"New cost: "<<best_cost);
      }

#ifdef GRAPH_DISPLAY_AVAILABLE
      if(verbose_ && disp_)
        disp_->displayPath(connecting_path_vector_.at(i),"graph_display",marker_color);
#endif
    }
  }

  if(solved)
  {
    // Set first and last nodes of the solution as the replanning and goal nodes (currently, they are the same configurations but different pointers)
    std::vector<ConnectionPtr>  connecting_path_conn = connecting_path_vector_.at(idx_best_sol)->getConnections();
    PathPtr new_path = concatWithNewPathToGoal(connecting_path_conn, node);
    replanned_path_ = new_path;
    double replanned_path_cost = replanned_path_->cost();

    assert(current_path_->findConnection(replanned_path_->getConnections().front()->getParent()->getConfiguration()) != nullptr);

    if(replanned_path_cost<current_cost)
    {
      success_ = true;
      if(verbose_)
        CNR_INFO(logger_,"Solution found! -> cost: " << replanned_path_cost);
    }
    else
    {
      success_ = false;
      if(verbose_)
        CNR_INFO(logger_,"Solution found but cost ("<<replanned_path_cost<<") is not better than the current cost (" << current_cost<<")");
    }
  }
  else
  {
    if(verbose_)
      CNR_ERROR(logger_,"New path not found!");
  }

  return success_;
}

PathPtr MPRRT::concatWithNewPathToGoal(const std::vector<ConnectionPtr>& connecting_path_conn,
                                       const NodePtr& path1_node)
{
  std::vector<ConnectionPtr> new_connecting_path_conn;
  NodePtr path2_node = std::make_shared<Node>(current_path_->getWaypoints().back());

  if(connecting_path_conn.size()>1)
  {
    NodePtr node1 = connecting_path_conn.front()->getChild();
    NodePtr node2 = connecting_path_conn.back()->getParent();

    ConnectionPtr conn1 = std::make_shared<Connection>(path1_node,node1,logger_,false);
    ConnectionPtr conn2 = std::make_shared<Connection>(node2,path2_node,logger_,false);

    conn1->setCost(connecting_path_conn.front()->getCost());
    conn2->setCost(connecting_path_conn.back()->getCost());

    conn1->add();
    conn2->add();

    new_connecting_path_conn.push_back(conn1);

    if(connecting_path_conn.size()>2)
      new_connecting_path_conn.insert(new_connecting_path_conn.end(), connecting_path_conn.begin()+1, connecting_path_conn.end()-1);

    new_connecting_path_conn.push_back(conn2);

    connecting_path_conn.front()->remove();
    connecting_path_conn.back()->remove();
  }
  else
  {
    ConnectionPtr conn1 = std::make_shared<Connection>(path1_node,path2_node,logger_,false);
    conn1->setCost(connecting_path_conn.front()->getCost());
    conn1->add();

    new_connecting_path_conn.push_back(conn1);
    connecting_path_conn.front()->remove();
  }

  return std::make_shared<Path>(new_connecting_path_conn,metrics_,checker_,logger_);
}

bool MPRRT::computeConnectingPath(const NodePtr &path1_node_fake,
                                  const NodePtr &path2_node_fake,
                                  const double &current_solution_cost,
                                  const double max_time,
                                  PathPtr &connecting_path,
                                  bool &directly_connected,
                                  TreeSolverPtr& solver)
{
  SamplerPtr sampler = std::make_shared<InformedSampler>(path1_node_fake->getConfiguration(), path2_node_fake->getConfiguration(), lb_, ub_,logger_,current_solution_cost);

  solver->setSampler(sampler);
  solver->resetProblem();
  solver->addStart(path1_node_fake);

  auto tic_solver = graph_time::now();
  solver->addGoal(path2_node_fake,max_time);
  auto toc_solver = graph_time::now();

  directly_connected = solver->solved();
  bool solver_has_solved;

  if(directly_connected)
  {
    connecting_path = solver->getSolution();
    solver_has_solved = true;
  }
  else
  {
    double solver_time = max_time-graph_duration(toc_solver-tic_solver).count();
    solver_has_solved = solver->solve(connecting_path,10000,solver_time);
  }

  return solver_has_solved;
}

bool MPRRT::replan()
{
  //Update the collision checkers for all the planning threads
  for(const RRTPtr& solver:solver_vector_)
    solver->setChecker(checker_->clone());

  //  moveit_msgs::PlanningScene scene_msg;
  //  checker_->getPlanningScene()->getPlanningSceneMsg(scene_msg);

  //  for(const RRTPtr& solver:solver_vector_)
  //    solver->getChecker()->setPlanningSceneMsg(scene_msg);

  //Replan
  ConnectionPtr conn = current_path_->findConnection(current_configuration_);
  NodePtr node_replan = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,false,is_a_new_node_);

  assert(current_path_->findConnection(node_replan->getConfiguration()) != nullptr);

  success_ = connect2goal(node_replan);

  if(not success_)
    replanned_path_ = current_path_;

  return success_;
}
}
