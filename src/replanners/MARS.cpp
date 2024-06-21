﻿#include <openmore/replanners/MARS.h>

namespace openmore
{
MARS::MARS(const Eigen::VectorXd& current_configuration,
           const PathPtr& current_path,
           const double& max_time,
           const TreeSolverPtr &solver,
           const cnr_logger::TraceLoggerPtr& logger): ReplannerBase(current_configuration,current_path,max_time,solver,logger)
{
  tree_ = current_path_->getTree();
  net_ = std::make_shared<Net>(tree_,logger_);

  copyTreeRoot();

  available_time_ =  std::numeric_limits<double>::infinity();
  pathSwitch_cycle_time_mean_ = std::numeric_limits<double>::infinity();
  time_percentage_variability_ = TIME_PERCENTAGE_VARIABILITY;

  reverse_start_nodes_ = false;
  full_net_search_ = true;

  an_obstacle_ = false;

  informedOnlineReplanning_disp_ = false;
  pathSwitch_disp_ = false;

  informedOnlineReplanning_verbose_ = false;
  pathSwitch_verbose_ = false;

  examined_flag_ = Node::getReservedFlagsNumber(); //the first free position in Node::flags_ vector where we can store our new custom flag
}

MARS::MARS(const Eigen::VectorXd& current_configuration,
           const PathPtr& current_path,
           const double& max_time,
           const TreeSolverPtr &solver,
           const cnr_logger::TraceLoggerPtr& logger,
           const std::vector<PathPtr> &other_paths): MARS(current_configuration,current_path,max_time,solver,logger)
{
  setOtherPaths(other_paths);
}

void MARS::copyTreeRoot()
{
  /* Net stops working when encounters the tree root, so doesn't allow to find a replanned path which pass through the start
   * So, a copy of the root (=start) is created and all the paths will start from this node and not from the root.*/

  if(not tree_)
    throw std::runtime_error("tree not defined!");

  paths_start_ = tree_->getRoot();
  assert(tree_->getRoot() == current_path_->getConnections().front()->getParent());
  NodePtr new_tree_root =std::make_shared<Node>(paths_start_->getConfiguration());

  ConnectionPtr conn = std::make_shared<Connection>(paths_start_,new_tree_root,logger_,false);
  conn->setCost(0.0);
  conn->add();

  assert(new_tree_root->getParentConnectionsSize() == 1);

  tree_->addNode(new_tree_root);
  if(not tree_->changeRoot(new_tree_root))
  {
    CNR_ERROR(logger_,"The root can't be moved to its copy");
    throw std::exception();
  }
}

bool MARS::mergePathToTree(const PathPtr &path)
{
  TreePtr path_tree = path->getTree();
  NodePtr path_goal = path->getConnections().back()->getChild();

  assert([&]() ->bool{
           for(const ConnectionPtr& conn:path->getConnections())
           {
             if(conn->isNet())
             return false;
           }
           return true;
         }());

  if(tree_ == path_tree)
    return true;

  assert(goal_node_->getParentConnectionsSize()== 1);

  //Merging the root
  if(not tree_)
  {
    if(path_tree)
    {
      tree_ = path_tree;
      copyTreeRoot();
    }
    else
    {
      std::vector<ConnectionPtr> conns;
      conns = current_path_->getConnections();
      conns.insert(conns.end(),path->getConnectionsConst().begin(),path->getConnectionsConst().end());

      double max_dist = 0;
      for(const ConnectionPtr& conn:conns)
      {
        if(conn->norm()>max_dist)
          max_dist = conn->norm();
      }

      tree_ = std::make_shared<Tree>(path->getNodes().front(),max_dist,checker_,metrics_,logger_);
      tree_->addBranch(path->getConnections());
      copyTreeRoot();
    }

    NodePtr current_path_start = current_path_->getConnections().front()->getParent();
    if(paths_start_->getConfiguration() == current_path_start->getConfiguration())
    {
      if(paths_start_ != current_path_start)
      {
        ConnectionPtr first_conn = current_path_->getConnections().front();
        assert(not first_conn->isNet());

        ConnectionPtr new_first_conn = std::make_shared<Connection>(paths_start_,first_conn->getChild(),logger_);
        new_first_conn->setCost(first_conn->getCost());
        new_first_conn->add();

        std::vector<ConnectionPtr> connections = current_path_->getConnections();
        connections.front() = new_first_conn;
        current_path_->setConnections(connections);

        first_conn->remove();

        assert(new_first_conn->getChild()->getParentConnectionsSize() == 1);
      }

      tree_->addBranch(current_path_->getConnections());

      current_path_->setTree(tree_);
      path->setTree(tree_);
    }
    else
    {
      CNR_ERROR(logger_,"Path has a starting node different from the tree root (!tree_)");
      tree_.reset();
      net_->getTree().reset();

      return false;
    }
  }
  else
  {
    NodePtr path_start = path->getConnections().front()->getParent();
    if(paths_start_->getConfiguration() == path_start->getConfiguration())
    {
      if(paths_start_ != path_start)
      {
        std::vector<ConnectionPtr> connections;
        if(path_tree)
        {
          assert(path_start == path_tree->getRoot());

          path_tree->addNode(paths_start_);
          if(not path_tree->changeRoot(path_goal))
            assert(0);

          NodePtr root = tree_->getRoot();
          if(not tree_->changeRoot(paths_start_))
            assert(0);

          ConnectionPtr conn;
          for(const ConnectionPtr& child_conn:path_start->getChildConnections())
          {
            assert(not child_conn->isNet());

            conn = std::make_shared<Connection>(paths_start_,child_conn->getChild(),logger_);
            conn->setCost(child_conn->getCost());
            conn->add();
          }

          assert(path_start->getParentConnectionsSize() == 1);
          for(const ConnectionPtr& parent_conn:path_start->getParentConnections())
          {
            assert(not parent_conn->isNet());

            conn = std::make_shared<Connection>(parent_conn->getParent(),paths_start_,logger_);
            conn->setCost(parent_conn->getCost());
            conn->add();
          }
          assert(paths_start_->getParentConnectionsSize() == 1);

          if(not path_tree->changeRoot(paths_start_))
            assert(0);
          if(not tree_->changeRoot(root))
            assert(0);

          path_tree->removeNode(path_start);

          assert([&]() ->bool{
                   std::vector<NodePtr> children = path_start->getChildren();
                   for(const NodePtr& n: children)
                   {
                     if(n->getParentConnectionsSize() != 1)
                     {
                       CNR_INFO(logger_,"n "<<*n);
                       return false;
                     }
                   }
                   return true;
                 }());

          connections = path_tree->getConnectionToNode(path_goal);
        }
        else
        {
          ConnectionPtr first_conn = path->getConnections().front();
          assert(not first_conn->isNet());

          ConnectionPtr new_first_conn = std::make_shared<Connection>(paths_start_,first_conn->getChild(),logger_);
          new_first_conn->setCost(first_conn->getCost());
          new_first_conn->add();

          first_conn->remove();

          assert(new_first_conn->getChild()->getParentConnectionsSize() == 1);

          connections = path->getConnections();
          connections.front() = new_first_conn;
        }

        path->setConnections(connections);
      }

      if(path_tree)
      {
        tree_->addTree(path_tree);
        path_tree = tree_;
      }
      else
        tree_->addBranch(path->getConnections());

      current_path_->setTree(tree_);
      path->setTree(tree_);
    }
    else
    {
      CNR_ERROR(logger_,"Path has a starting node different from the tree root (tree_)");
      return false;
    }
  }

  //Merging the goal
  std::vector<ConnectionPtr> path_conns = path->getConnections();
  ConnectionPtr goal_conn = path->getConnections().back();

  assert(goal_node_->getParentConnectionsSize() == 1);

  ConnectionPtr new_goal_conn;
  (goal_node_->getParentConnectionsSize() == 0)?
        (new_goal_conn = std::make_shared<Connection>(goal_conn->getParent(),goal_node_,logger_,false)):
        (new_goal_conn = std::make_shared<Connection>(goal_conn->getParent(),goal_node_,logger_,true ));

  new_goal_conn->setCost(goal_conn->getCost());
  new_goal_conn->add();

  goal_conn->remove();

  assert(goal_node_->getParentConnectionsSize() == 1);

  path_conns.back() = new_goal_conn;
  path->setConnections(path_conns);

  tree_->removeNode(path_goal);
  net_->setTree(tree_);

  assert(goal_node_->getParentConnectionsSize() == 1);

  return true;
}

void MARS::clearInvalidConnections()
{
  /* Set the cost of subtree connections equal to their default value. In previous call to MARS replanner
   * the cost of some subtrees connections has been set equal to infinity. These connections usually are
   * not checked because not part of a path. So, now reset their costs. The connections of the paths are excluded
   * because their costs have been updated by an external collision checker */

  std::vector<PathPtr> paths = other_paths_;
  paths.push_back(current_path_);
  paths.push_back(replanned_path_);

  std::vector<ConnectionPtr> connections;
  for(const PathPtr& p:paths)
    connections.insert(connections.end(),p->getConnectionsConst().begin(),p->getConnectionsConst().end());

  for(invalid_connection_ptr& invalid_conn:invalid_connections_)
  {
    assert(invalid_conn->connection->isRecentlyChecked());

    if(std::find(connections.begin(),connections.end(),invalid_conn->connection)>=connections.end())
      invalid_conn->connection->setCost(invalid_conn->cost);
  }

  invalid_connections_.clear();
}

std::vector<PathPtr> MARS::addAdmissibleCurrentPath(const size_t &idx_current_conn, PathPtr& admissible_current_path)
{
  std::vector<PathPtr> reset_other_paths;
  admissible_current_path = nullptr;

  if(current_path_->getCostFromConf(current_configuration_) == std::numeric_limits<double>::infinity())
  {
    if(current_path_->getConnections().back()->getCost() == std::numeric_limits<double>::infinity())
      return other_paths_;
    else
    {
      size_t z = current_path_->getConnectionsSize()-2;  //penultimate connection (last connection is at end-1)
      ConnectionPtr conn;

      while(z>=idx_current_conn) //to find the savable part of current_path, the subpath after the connection obstruced by the obstacle
      {
        conn = current_path_->getConnections().at(z);
        if(conn->getCost() == std::numeric_limits<double>::infinity())
        {
          admissible_current_path = current_path_->getSubpathFromNode(conn->getChild());
          break;
        }
        z -= 1;
      }
    }

    /* Adding the savable subpath of the current_path to the set of available paths */
    if(admissible_current_path)
      reset_other_paths.push_back(admissible_current_path);

    reset_other_paths.insert(reset_other_paths.end(),other_paths_.begin(),other_paths_.end());

    return reset_other_paths;
  }
  else
  {
    if(idx_current_conn<current_path_->getConnectionsSize()-1)
    {
      admissible_current_path = current_path_->getSubpathFromNode(current_path_->getConnections().at(idx_current_conn)->getChild());
      reset_other_paths.push_back(admissible_current_path);
      reset_other_paths.insert(reset_other_paths.end(),other_paths_.begin(),other_paths_.end());
    }
    else
      reset_other_paths = other_paths_;

    return reset_other_paths;
  }
}

std::vector<ps_goal_ptr> MARS::sortNodes(const NodePtr& start_node)
{
  /* Sort nodes based on the metrics utopia. In the case of Euclidean metrics, nodes are sorted based on the disance from start_node.
   * Prioritize nodes with free subpath to goal:
   *  - firstly, consider nodes with free subpath
   *  - then, consider nodes with invalid subpath (later the net will be used to search for a better subpath, if full_neat_search_ is true)
   */

  PathPtr tmp_path;
  std::vector<NodePtr> nodes;
  ps_goal_ptr pathswitch_goal;
  std::vector<ps_goal_ptr> goals;
  std::vector<NodePtr> considered_nodes;
  std::multimap<double,ps_goal_ptr> ps_goals_map, ps_invalid_goals_map;

  double euclidean_distance, utopia;
  bool start_node_belongs_to_p;
  bool goal_node_considered = false;
  for(const PathPtr& p:admissible_other_paths_)
  {
    nodes = p->getNodes();
    if(goal_node_considered)
    {
      assert(nodes.back() == goal_node_);
      nodes.pop_back();
    }

    if(std::find(nodes.begin(),nodes.end(),start_node)<nodes.end())
      start_node_belongs_to_p = true;
    else
      start_node_belongs_to_p = false;

    for(const NodePtr& n:nodes)
    {
      if(std::find(considered_nodes.begin(),considered_nodes.end(),n)<considered_nodes.end())
        continue;

      euclidean_distance = (start_node->getConfiguration(),n->getConfiguration()).norm();
      utopia = metrics_->utopia(start_node->getConfiguration(),n->getConfiguration());

      if(utopia<TOLERANCE)
        continue;

      if(start_node_belongs_to_p)
      {
        // Do not connect nodes which are on the same path and already connected by a straight connection (only if there are no obstacles in between)
        tmp_path = p->getSubpathFromNode(start_node);
        tmp_path = tmp_path->getSubpathToNode(n);
        if(tmp_path->cost()<std::numeric_limits<double>::infinity())
        {
          if(std::abs(tmp_path->computeEuclideanNorm()-euclidean_distance)<TOLERANCE)
          {
            if(n == goal_node_)
              goal_node_considered = true;

            if(pathSwitch_verbose_)
              CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::RED()<<"node removed from Q2 list: "<<n<<cnr_logger::RED());

            continue;
          }
        }
      }

      pathswitch_goal = std::make_shared<ps_goal>();
      pathswitch_goal->node = n;
      pathswitch_goal->utopia = utopia;

      if(n != goal_node_)
      {
        pathswitch_goal->subpath = p->getSubpathFromNode(n);
        pathswitch_goal->subpath_cost = pathswitch_goal->subpath->cost();
      }
      else
      {
        goal_node_considered = true;
        pathswitch_goal->subpath = nullptr;
        pathswitch_goal->subpath_cost = 0.0;
      }

      considered_nodes.push_back(n);

      if(pathswitch_goal->subpath_cost<std::numeric_limits<double>::infinity())
        ps_goals_map.insert(std::pair<double,ps_goal_ptr>(utopia,pathswitch_goal));
      else
      {
        if(full_net_search_)
          ps_invalid_goals_map.insert(std::pair<double,ps_goal_ptr>(utopia,pathswitch_goal));
      }
    }
  }

  //Firstly, add the valid goals, ordered by utopia
  for(const std::pair<double,ps_goal_ptr> &p: ps_goals_map)
    goals.push_back(p.second);

  //Then, add the invalid goals, ordered by utopia (void map if full_net_search_ == false)
  for(const std::pair<double,ps_goal_ptr> &p: ps_invalid_goals_map)
    goals.push_back(p.second);

  return goals;
}

std::vector<NodePtr> MARS::startNodes(const std::vector<ConnectionPtr>& subpath1_conn)
{
  std::vector<NodePtr> start_node_vector;

  if((subpath1_conn.front()->getCost() == std::numeric_limits<double>::infinity()))
  {
    /* If the current conf is obstructed the replanning will start from the current node */
    NodePtr current_node = subpath1_conn.front()->getParent();

    assert(current_node->getParentConnectionsSize() == 1);

    if(not current_node->getFlag(examined_flag_,false))
      start_node_vector.push_back(current_node);
  }
  else
  {
    /* If the current connection is free, all the nodes between the current child to the parent
     * of the connection obstructed are considered as starting points for the replanning */

    for(const ConnectionPtr& conn:subpath1_conn)
    {
      if(conn == subpath1_conn.front())
        continue;
      else if(conn == subpath1_conn.back())
      {
        /* If the path is free, you can consider all the nodes but it is useless to consider
         * the last one before the goal (it is already connected to the goal with a straight line) */

        if(conn->getCost() == std::numeric_limits<double>::infinity() && (not conn->getParent()->getFlag(examined_flag_,false)))
          start_node_vector.push_back(conn->getParent());
      }
      else
      {
        if(not conn->getParent()->getFlag(examined_flag_,false))
          start_node_vector.push_back(conn->getParent());

        if(conn->getCost() ==  std::numeric_limits<double>::infinity())
          break;
      }
    }
  }

  assert([&]() ->bool{
           for(const NodePtr& n:start_node_vector)
           {
             if(n->getParentConnectionsSize() != 1)
             {
               CNR_WARN(logger_,"n: "<<n<<" "<<*n);

               for(const NodePtr& nn:start_node_vector)
               {
                 CNR_INFO(logger_,"nn: "<<nn<<" "<<*nn);
               }

               for(const ConnectionPtr& c:subpath1_conn)
               {
                 CNR_INFO(logger_,*c);
               }
               return false;
             }
           }
           return true;
         }());

  if(reverse_start_nodes_)
    std::reverse(start_node_vector.begin(),start_node_vector.end());

  if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
    CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"NEW J: "<<(int) (start_node_vector.size()-1)<<cnr_logger::RESET());

  return start_node_vector;
}

bool MARS::findValidSolution(const std::multimap<double,std::vector<ConnectionPtr>> &map, const double &cost2beat, std::vector<ConnectionPtr> &solution, double& cost, bool verbose)
{
  unsigned int number_of_candidates = 0;
  return findValidSolution(map,cost2beat,solution,cost,number_of_candidates,verbose);
}

bool MARS::findValidSolution(const std::multimap<double,std::vector<ConnectionPtr>> &map, const double &cost2beat, std::vector<ConnectionPtr> &solution, double& cost, unsigned int &number_of_candidates, bool verbose)
{
  solution.clear();
  number_of_candidates = 0;

  if(not map.empty())
  {
    if(verbose)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::CYAN()<<"Map not empty, size "<<map.size()<<cnr_logger::RESET());

    bool free;
    int i,size;
    double updated_cost;

    for(const std::pair<double,std::vector<ConnectionPtr>> &solution_pair:map)
    {
      if(solution_pair.first == std::numeric_limits<double>::infinity())
      {
        updated_cost = std::numeric_limits<double>::infinity();

        if(verbose)
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::CYAN()<<"solution cost inf -> updated cost inf"<<cnr_logger::RESET());

        assert([&]() ->bool{
                 i=0;
                 double updated_cost_check = 0.0;
                 size = solution_pair.second.size();

                 while(updated_cost_check<std::numeric_limits<double>::infinity() && i<size)
                 {
                   updated_cost_check += solution_pair.second.at(i)->getCost();
                   if(solution_pair.second.at(i)->getCost() == std::numeric_limits<double>::infinity() && not solution_pair.second.at(i)->isRecentlyChecked())
                   {
                     CNR_INFO(logger_,"no recently checked");
                     return false;
                   }
                   i++;
                 }

                 if(updated_cost_check == updated_cost)
                 return true;
                 else
                 return false;
               }());
      }
      else //some connections are shared between solutions and during checking some of them can be set to infinity cost -> update cost
      {
        if(verbose)
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::CYAN()<<"solution cost not inf, updating the cost.."<<cnr_logger::RESET());

        i=0;
        updated_cost = 0.0;
        size = solution_pair.second.size();

        while(updated_cost<std::numeric_limits<double>::infinity() && i<size)
        {
          if(verbose)
            CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::CYAN()<<"connection: "<<solution_pair.second.at(i)<<" cost: "<<solution_pair.second.at(i)->getCost()<<cnr_logger::RESET());

          updated_cost += solution_pair.second.at(i)->getCost();
          assert([&]() ->bool{
                   if(solution_pair.second.at(i)->getCost() == std::numeric_limits<double>::infinity())
                   {
                     if(solution_pair.second.at(i)->isRecentlyChecked())
                     {
                       return true;
                     }
                     else
                     {
                       CNR_INFO(logger_,"cost inf but not recently checked");
                       return false;
                     }
                   }
                   return true;
                 }());
          i++;
        }
      }

      if(verbose)
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::CYAN()<<"updated cost: "<<updated_cost<<" cost2beat: "<<cost2beat<<cnr_logger::RESET());

      if(updated_cost<cost2beat)
      {
        if(verbose)
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::CYAN()<<"new candidate solution"<<cnr_logger::RESET());

        number_of_candidates++;

        free = true;
        for(const ConnectionPtr& conn: solution_pair.second)
        {
          if(not conn->isRecentlyChecked())
          {
            conn->setRecentlyChecked(true);
            flagged_connections_.push_back(conn);

            if(verbose)
              CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::CYAN()<<"conn "<<conn<<" not recently checked"<<cnr_logger::RESET());

            assert(conn->getCost() != std::numeric_limits<double>::infinity());

            if(not checker_->checkConnection(conn))
            {
              free = false;

              /* Save the invalid connection */
              invalid_connection_ptr invalid_conn = std::make_shared<invalid_connection>();
              invalid_conn->connection = conn;
              invalid_conn->cost = conn->getCost();
              invalid_connections_.push_back(invalid_conn);

              /* Set the cost equal to infinity */
              conn->setCost(std::numeric_limits<double>::infinity());

              if(verbose)
                CNR_INFO(logger_,"conn "<<conn<<" obstructed!");

              break;
            }
          }
          else
          {
            if(verbose)
              CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::CYAN()<<"conn "<<conn<<" already checked, cost: "<<conn->getCost()<<cnr_logger::RESET());

            assert(std::find(flagged_connections_.begin(),flagged_connections_.end(),conn)<flagged_connections_.end());

            if(conn->getCost() == std::numeric_limits<double>::infinity()) //it should not happen..
            {
              assert(0);
              free = false;
              break;
            }
          }
        }

        if(free)
        {
          if(verbose)
            CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::CYAN()<<"Solution free, cost: "<<updated_cost<<cnr_logger::RESET());

          solution = solution_pair.second;
          cost = updated_cost;

          //updated_cost != solution_pair.first can happen only with updated_cost == infinity and solution_pair.first not
          assert(cost < std::numeric_limits<double>::infinity());
          assert([&]() ->bool{
                   if(std::abs(updated_cost-solution_pair.first)>NET_ERROR_TOLERANCE)
                   {
                     CNR_INFO(logger_,"updated cost "<<updated_cost<<" solution_pair.first "<<solution_pair.first <<" NET_ERROR_TOLERANCE "<<NET_ERROR_TOLERANCE);
                     return false;
                   }
                   return true;
                 }());

          return true;
        }
      }
      else
      {
        if(verbose)
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::CYAN()<<"not a candidate solution"<<cnr_logger::RESET());

        if(updated_cost<std::numeric_limits<double>::infinity()) //solutions ordered by cost in the map, so, if this solution is not obstructed and it is worst than cost2beat, no better solutions exist (subsequent solutions will have higher cost or cost infinite)
        {
          if(verbose)
            CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::CYAN()<<"update cost not infinite, no better solutions available -> exit"<<cnr_logger::RESET());

          return false;
        }
      }

      if(verbose)
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::CYAN()<<"-------------------------"<<cnr_logger::RESET());
    }
  }
  return false;
}

PathPtr MARS::bestExistingSolution(const PathPtr& current_solution)
{
  std::multimap<double,std::vector<ConnectionPtr>> tmp_map;
  return bestExistingSolution(current_solution,tmp_map);
}
PathPtr MARS::bestExistingSolution(const PathPtr& current_solution, std::multimap<double,std::vector<ConnectionPtr>>& tmp_map)
{
  tmp_map.clear();
  PathPtr solution;

  NodePtr current_node = current_solution->getStartNode();
  double best_cost = current_solution->cost();

  auto tic = graph_time::now();
  tmp_map = net_->getConnectionBetweenNodes(current_node,goal_node_,best_cost);

  if(informedOnlineReplanning_verbose_)
    CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::CYAN()<<tmp_map.size()<<" solutions with lower cost found in "
             <<graph_duration(graph_time::now()-tic).count()<<" seconds!"<<cnr_logger::RESET());

#ifdef ROS_AVAILABLE
  if(informedOnlineReplanning_disp_)
  {
    disp_->changeNodeSize({0.025,0.025,0.025});
    disp_->changeConnectionSize({0.025,0.025,0.025});

    int id;
    int n_sol = 0;
    for(const std::pair<double,std::vector<ConnectionPtr>> &solution_pair:tmp_map)
    {
      PathPtr disp_path = std::make_shared<Path>(solution_pair.second,metrics_,checker_);

      n_sol++;
      id = disp_->displayPath(disp_path);
      disp_->nextButton("Displaying the "+std::to_string(n_sol)+"° existing solution (cost " + std::to_string(disp_path->cost())+")");
      disp_->clearMarker(id);
    }
    disp_->defaultNodeSize();
    disp_->defaultConnectionSize();
  }
#endif

  double new_cost;
  std::vector<ConnectionPtr> solution_conns;
  tic = graph_time::now();
  findValidSolution(tmp_map,best_cost,solution_conns,new_cost)?
        (solution = std::make_shared<Path>(solution_conns,metrics_,checker_,logger_)):
        (solution = current_solution);

  if(informedOnlineReplanning_verbose_ && not tmp_map.empty())
    CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::CYAN()<<"Solutions checked in "<<graph_duration(graph_time::now()-tic).count()<<" seconds!");

  solution->setTree(tree_);
  return solution;
}

void MARS::simplifyAdmissibleOtherPaths(const PathPtr& current_solution_path, const NodePtr& start_node, const std::vector<PathPtr>& reset_other_paths)
{
  if(start_node == current_solution_path->getGoalNode())
    return;

  std::vector<PathPtr> paths;
  std::vector<NodePtr> nodes;
  for(unsigned int i=0;i<reset_other_paths.size();i++)
  {
    nodes = reset_other_paths[i]->getNodes();
    if(std::find(nodes.begin(),nodes.end(),start_node)<nodes.end())
      paths.push_back(reset_other_paths[i]->getSubpathFromNode(start_node));
    else
      paths.push_back(reset_other_paths[i]);
  }

  admissible_other_paths_.clear();
  admissible_other_paths_ = paths;

  assert(admissible_other_paths_.size() == reset_other_paths.size());
}

double MARS::maxSolverTime(const graph_time_point& tic, const graph_time_point& tic_cycle)
{
  double time;
  auto toc = graph_time::now();
  double max_time = pathSwitch_max_time_-graph_duration(toc-tic).count();

  if(pathSwitch_disp_)
    time = std::numeric_limits<double>::infinity();
  else if(pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity() || an_obstacle_)
    time = max_time; //when there is an obstacle or when the cycle time mean has not been defined yet
  else
    time = std::min((2-time_percentage_variability_)*pathSwitch_cycle_time_mean_-graph_duration(toc-tic_cycle).count(),max_time);

  if(time<0.0)
    time = 0.0;

  return time;
}

void MARS::convertToSubtreeSolution(const PathPtr& net_solution, const std::vector<NodePtr>& black_nodes)
{
  std::vector<ConnectionPtr> connections = net_solution->getConnections();
  connections.pop_back(); //the last connection must remain a net connection

  NodePtr node;
  for(ConnectionPtr& conn:connections)
  {
    if(conn->isNet())
    {
      node = conn->getChild();
      assert(std::find(black_nodes.begin(),black_nodes.end(),node)>=black_nodes.end());

      if(not node->switchParentConnection(conn))
        assert(0);
    }
  }
}

bool MARS::computeConnectingPath(const NodePtr& path1_node, const NodePtr& path2_node, const double& diff_subpath_cost, const PathPtr& current_solution, const graph_time_point &tic, const graph_time_point &tic_cycle, PathPtr& connecting_path, bool& quickly_solved)
{
  connecting_path = nullptr;

  /* Create a subtree rooted at path1_node. It will be used to build the connecting path between path1_node and path2_node */
  std::vector<NodePtr> black_list;
  std::vector<PathPtr> paths = other_paths_;
  paths.push_back(current_path_);
  paths.push_back(current_solution);

  for(const PathPtr& p:paths)
  {
    std::vector<NodePtr> nodes = p->getNodes();
    black_list.insert(black_list.end(),nodes.begin(),nodes.end());
  }
  assert(not black_list.empty());

  SubtreePtr subtree = Subtree::createSubtree(tree_,path1_node,
                                              path2_node->getConfiguration(),
                                              diff_subpath_cost,
                                              black_list,true); //collision check before adding a node
  assert([&]() ->bool{
           std::vector<NodePtr> leaves;
           subtree->getLeaves(leaves);

           for(const NodePtr& n:leaves)
           {
             std::vector<ConnectionPtr> conns = subtree->getConnectionToNode(n);
             for(const ConnectionPtr& c:conns)
             {
               if(c->getCost() == std::numeric_limits<double>::infinity())
               return false;
             }
           }
           return true;
         }());

#ifdef ROS_AVAILABLE
  if(pathSwitch_disp_)
  {
    disp_->changeConnectionSize({0.025,0.025,0.025});
    int subtree_id = disp_->displaySubtree(subtree,"pathplan",{0.0,0.0,0.0,1.0});
    disp_->nextButton("Displaying subtree..");
    disp_->clearMarker(subtree_id);
    disp_->defaultConnectionSize();
  }
#endif

  /* Search for an already existing solution between path1_node and path2_node */
  if(pathSwitch_verbose_)
    CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::YELLOW()<<"Searching for an already existing solution in the subtree.."<<cnr_logger::RESET());

  auto tic_search = graph_time::now();

  bool search_in_subtree = true;
  double net_time = maxSolverTime(tic,tic_cycle);

  NetPtr net = std::make_shared<Net>(subtree,logger_);
  std::multimap<double,std::vector<ConnectionPtr>> already_existing_solutions_map = net->getConnectionBetweenNodes(path1_node,path2_node,diff_subpath_cost,
                                                                                                                   black_list,net_time,search_in_subtree);
  double time_search = graph_duration(graph_time::now()-tic_search).count();

  if(pathSwitch_verbose_)
    CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::YELLOW()<<"In the subtree exist "<< already_existing_solutions_map.size()
             <<" paths to path2_node (time to search "<<time_search<<" s, max time "<<net_time<<"s )"<<cnr_logger::RESET());

  tic_search = graph_time::now();
  unsigned int number_of_candidates = 0;
  double already_existing_solution_cost;
  std::vector<ConnectionPtr> already_existing_solution_conn;
  if(findValidSolution(already_existing_solutions_map,diff_subpath_cost,already_existing_solution_conn,
                       already_existing_solution_cost,number_of_candidates,false))
  {
    connecting_path = std::make_shared<Path>(already_existing_solution_conn,metrics_,checker_,logger_);
    connecting_path->setTree(tree_);
    quickly_solved = true;

    assert(already_existing_solution_cost == connecting_path->cost());

    if(pathSwitch_verbose_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BOLDYELLOW()<<"A solution with cost "
               << already_existing_solution_cost<<" has been found in the subtree in "<<graph_duration(graph_time::now()-tic_search).count()
               <<" seconds! Making it a solution of the subtree.."<<cnr_logger::RESET());

    convertToSubtreeSolution(connecting_path,black_list);

#ifdef ROS_AVAILABLE
    if(pathSwitch_disp_)
    {
      disp_->changeConnectionSize({0.02,0.02,0.02});
      int connecting_path_id = disp_->displayPath(connecting_path,"pathplan",{1.0,0.4,0.0,1.0});
      disp_->nextButton("Displaying connecting_path..");
      disp_->clearMarker(connecting_path_id);
      disp_->defaultConnectionSize();
    }
#endif

    return true;
  }
  else
  {
    if(pathSwitch_verbose_)
    {
      if(number_of_candidates>0)
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::YELLOW()<<number_of_candidates
                 << " candidate solutions found in the subtree but no one was free (check time "
                 <<graph_duration(graph_time::now()-tic_search).count()<<" seconds)"<<cnr_logger::RESET());
      else
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::YELLOW()<<"No candidate solutions found in the subtree (search time "
                 <<graph_duration(graph_time::now()-tic_search).count()<<" seconds)"<<cnr_logger::RESET());
    }

    //Remove the invalid branches from the subtree
    for(const std::pair<double,std::vector<ConnectionPtr>>& p:already_existing_solutions_map)
    {
      for(const ConnectionPtr& c:p.second)
      {
        if(c->getCost() == std::numeric_limits<double>::infinity())
          subtree->hideFromSubtree(c->getChild());
      }
    }
  }

  /* If no solutions already exist, search for a new one. The ellipsoide determined
   * by diff_subpath_cost is used to sample the space. Outside of this ellipsoid,
   * the nodes would create an inconvenient connecting_path */

  SamplerPtr sampler = std::make_shared<InformedSampler>(path1_node->getConfiguration(),
                                                         path2_node->getConfiguration(),
                                                         lb_, ub_,logger_,diff_subpath_cost);

  std::vector<NodePtr> subtree_nodes;
  NodePtr path2_node_fake = std::make_shared<Node>(path2_node->getConfiguration());

  bool solver_has_solved = false;
  bool valid_connecting_path_found = false;

  double solver_time = maxSolverTime(tic,tic_cycle);
  double available_search_time = solver_time;
  auto tic_before_search = graph_time::now();

  while(available_search_time>0)
  {
    solver_->resetProblem();
    solver_->setSampler(sampler);
    solver_->addStart(path1_node);
    solver_->addStartTree(subtree); //after addStart

    connecting_path = nullptr;
    subtree_nodes = subtree->getNodes(); //nodes already in the subtree

    available_search_time = solver_time-graph_duration(graph_time::now()-tic_before_search).count();
    if(pathSwitch_verbose_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::YELLOW()<<"Searching for a connecting path...max time: "<<available_search_time<<cnr_logger::RESET());

    auto tic_solver = graph_time::now();
    solver_->addGoal(path2_node_fake,available_search_time*0.85);
    auto toc_solver = graph_time::now();

    quickly_solved = solver_->solved();

    if(quickly_solved)
    {
      solver_has_solved = true;
      connecting_path = solver_->getSolution();
    }
    else
    {
      available_search_time = solver_time-graph_duration(graph_time::now()-tic_before_search).count();

      if(pathSwitch_verbose_)
      {
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::YELLOW()<<"Direct connection NOT found (time "<<graph_duration(toc_solver-tic_solver).count()<<" s)"<<cnr_logger::RESET());
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::YELLOW()<<"Solving...max time: "<<available_search_time<<cnr_logger::RESET());
      }

      tic_solver = graph_time::now();
      solver_has_solved = solver_->solve(connecting_path,10000,available_search_time*0.85);
      toc_solver = graph_time::now();
    }

    if(solver_has_solved)
    {
      if(pathSwitch_verbose_)
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::YELLOW()<<"Solved in "<<graph_duration(toc_solver-tic_solver).count()
                 <<" s (direct connection to goal: "<<quickly_solved<<")"<<cnr_logger::RESET());

      bool subtree_valid = true;
      ConnectionPtr obstructed_connection = nullptr;
      for(const ConnectionPtr& c:connecting_path->getConnections())
      {
        available_search_time = solver_time-graph_duration(graph_time::now()-tic_before_search).count();
        if(available_search_time<=0.0)
        {
          subtree_valid = false;
          quickly_solved = false;
          break;
        }

        std::vector<NodePtr>::iterator it = std::find(subtree_nodes.begin(),subtree_nodes.end(),c->getChild());

        //lazy collision check of the connections that already were in the subtree
        if(it<subtree_nodes.end())
        {
          if(not subtree_valid)
            continue;
          if(c->isRecentlyChecked())
          {
            if(c->getCost() == std::numeric_limits<double>::infinity())
            {
              CNR_INFO(logger_,*c);
              assert(0);
            }
            else
              continue;
          }

          if(not checker_->checkConnection(c))
          {
            invalid_connection_ptr invalid_conn = std::make_shared<invalid_connection>();
            invalid_conn->connection = c;
            invalid_conn->cost = c->getCost();
            invalid_connections_.push_back(invalid_conn);

            c->setCost(std::numeric_limits<double>::infinity());

            obstructed_connection = c;
            subtree_valid = false;
            quickly_solved = false;
          }
          else
            c->setCost(metrics_->cost(c->getParent(),c->getChild()));

          if(not c->isRecentlyChecked())
          {
            c->setRecentlyChecked(true);
            flagged_connections_.push_back(c);
          }
        }
        else //do not re-check connections just added to the subtree
        {
          assert(not c->isRecentlyChecked());
          c->setRecentlyChecked(true);
          flagged_connections_.push_back(c);
        }
      }

      if(subtree_valid)
      {
        assert([&]() ->bool{
                 PathPtr clone = connecting_path->clone();
                 if(connecting_path->isValid())
                 return true;

                 CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BOLDYELLOW()<<"clone "<<*clone<<cnr_logger::RESET());
                 CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BOLDRED()<<"connecting path "<<*connecting_path<<cnr_logger::RESET());
                 for(const NodePtr& n:subtree_nodes)
                 CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BOLDWHITE()<<"subtree node "<<n->getConfiguration().transpose()<<" ("<<n<<")"<<cnr_logger::RESET());

                 return false;
               }());

        valid_connecting_path_found = true;
        break;
      }
      else
      {
        subtree->purgeFromHere(path2_node_fake);

        if(obstructed_connection != nullptr)
          subtree->hideFromSubtree(obstructed_connection->getChild());

        if(pathSwitch_verbose_)
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::YELLOW()<<"Lazy check detected that subtree was not valid, compute again.."<<cnr_logger::RESET());
      }
    }
    else
    {
      if(pathSwitch_verbose_)
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::YELLOW()<<"Not solved, time: "<<graph_duration(toc_solver-tic_solver).count()<<cnr_logger::RESET());

      break;
    }

    available_search_time = solver_time-graph_duration(graph_time::now()-tic_before_search).count();
  }

  if(valid_connecting_path_found)
  {
    /* Search for the best solution in the subtree which connects path1_node to path2_node_fake */
    assert(path2_node_fake->getParentConnectionsSize   () == 1);
    assert(path2_node_fake->getNetChildConnectionsSize () == 0);
    assert(path2_node_fake->getNetParentConnectionsSize() == 0);

    net_time = maxSolverTime(tic,tic_cycle);

    number_of_candidates = 0;
    double connecting_path_cost;
    std::vector<ConnectionPtr> connecting_path_conn;

    auto tic_net = graph_time::now();
    std::multimap<double,std::vector<ConnectionPtr>> connecting_paths_map = net->getConnectionBetweenNodes(path1_node,path2_node_fake,diff_subpath_cost,
                                                                                                           black_list,net_time,search_in_subtree);
    if(pathSwitch_verbose_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::YELLOW()<<"Net search in the subtree found "
               <<connecting_paths_map.size()<<" solutions in "<<graph_duration(graph_time::now()-tic_net).count()<<" s"<<cnr_logger::RESET());

    /*To be sure map contains the connecting path computed*/
    double cost = 0.0;
    std::vector<ConnectionPtr> conns = subtree->getConnectionToNode(path2_node_fake);
    for(const ConnectionPtr& c: conns)
      cost += c->getCost();

    if(cost<diff_subpath_cost)
    {
      std::pair<double,std::vector<ConnectionPtr>> p;
      p.first = cost;
      p.second = conns;

      connecting_paths_map.insert(p);
    }

    if(not connecting_paths_map.empty())
    {
      if(findValidSolution(connecting_paths_map,diff_subpath_cost,connecting_path_conn,connecting_path_cost,number_of_candidates))
      {
        ConnectionPtr last_conn = connecting_path_conn.back();

        assert(last_conn != nullptr);
        assert(last_conn->getChild() == path2_node_fake);

        ConnectionPtr new_conn= std::make_shared<Connection>(last_conn->getParent(),path2_node,logger_,(path2_node->getParentConnectionsSize()>0));
        new_conn->setCost(last_conn->getCost());
        new_conn->add();

        assert(path2_node->getParentConnectionsSize() == 1);

        connecting_path_conn.back() = new_conn;
        connecting_path = std::make_shared<Path>(connecting_path_conn,metrics_,checker_,logger_);
        connecting_path->setTree(tree_);

        assert(last_conn->isRecentlyChecked());
        new_conn->setRecentlyChecked(last_conn->isRecentlyChecked());

        std::vector<ConnectionPtr>::iterator it = std::find(flagged_connections_.begin(),flagged_connections_.end(),last_conn);
        assert(it<flagged_connections_.end());
        *it = new_conn;

        last_conn->remove();

        assert(connecting_path->cost() == std::numeric_limits<double>::infinity() || connecting_path->cost()<diff_subpath_cost);

#ifdef ROS_AVAILABLE
        if(pathSwitch_disp_)
        {
          disp_->changeConnectionSize({0.02,0.02,0.02});
          int connecting_path_id = disp_->displayPath(connecting_path,"pathplan",{1.0,0.4,0.0,1.0});
          disp_->nextButton("Displaying connecting_path..");
          disp_->clearMarker(connecting_path_id);
          disp_->defaultConnectionSize();
        }
#endif

        if(not((path2_node != tree_->getRoot() && path2_node->getParentConnectionsSize() == 1) ||
               (path2_node == tree_->getRoot() && path2_node->getParentConnectionsSize() == 0)))
        {
          CNR_INFO(logger_,"path2_node "<<path2_node);
          CNR_INFO(logger_,*path2_node);

          CNR_INFO(logger_,"root "<<tree_->getRoot());
          CNR_INFO(logger_,*tree_->getRoot());

          CNR_INFO(logger_,"fake root "<<paths_start_);
          CNR_INFO(logger_,*paths_start_);

          throw std::exception();
        }

        subtree->purgeFromHere(path2_node_fake); //disconnect and remove the fake node
        assert(not tree_->isInTree(path2_node_fake));

        assert([&]() ->bool{
                 if(connecting_path->isValid())
                 {
                   return true;
                 }
                 else
                 {
                   CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BOLDRED()<<"connecting path "<<*connecting_path<<cnr_logger::RESET());
                   return false;
                 }
               }());

        return true;
      }
      else
      {
        if(pathSwitch_verbose_)
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::YELLOW()<<"No free solutions found in the subtree"<<cnr_logger::RESET());
      }
    }
    else
    {
      if(pathSwitch_verbose_)
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::YELLOW()<<"No solutions with cost less than diff_subpath_cost found in the subtree"<<cnr_logger::RESET());
    }
  }

  /* If a solution was not found or the found solution was not free */
  connecting_path = nullptr;

  subtree->purgeFromHere(path2_node_fake); //disconnect and remove the fake node
  assert(not tree_->isInTree(path2_node_fake));

  return false;
}

bool MARS::pathSwitch(const PathPtr &current_path,
                      const NodePtr &path1_node,
                      PathPtr &new_path)
{
  auto tic=graph_time::now();
  graph_time_point toc, tic_cycle, toc_cycle;

  (pathSwitch_disp_ == true)?
        (pathSwitch_max_time_ = std::numeric_limits<double>::infinity()):
        (pathSwitch_max_time_ = available_time_);

  double time = pathSwitch_max_time_;
  std::vector<double> time_vector;

  if(pathSwitch_verbose_)
    CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"PathSwitch cycle time mean: "<<pathSwitch_cycle_time_mean_<<cnr_logger::RESET());

  if(pathSwitch_cycle_time_mean_ != std::numeric_limits<double>::infinity())
    time_vector.push_back(pathSwitch_cycle_time_mean_);

  if(not pathSwitch_disp_)
  {
    if(pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity() || an_obstacle_)
    {
      if(time<=0.0)
        return false;
    }
    else
    {
      if(time<time_percentage_variability_*pathSwitch_cycle_time_mean_)
        return false;
    }
  }

#ifdef ROS_AVAILABLE
  int new_node_id;
  std::vector<int> node_id_vector;
#endif

  bool success = false;
  NodePtr path1_node_of_sol, path2_node_of_sol;

  PathPtr path1_subpath = current_path->getSubpathFromNode(path1_node);
  double candidate_solution_cost = path1_subpath->cost();

  std::vector<ps_goal_ptr> ordered_ps_goals = sortNodes(path1_node);
  int remaining_goals = ordered_ps_goals.size();

  for(const ps_goal_ptr& ps_goal:ordered_ps_goals)
  {
    tic_cycle = graph_time::now();

    NodePtr path2_node = ps_goal->node;
    PathPtr path2_subpath = ps_goal->subpath;
    double path2_subpath_cost = ps_goal->subpath_cost;

    remaining_goals--;

    if(pathSwitch_disp_ || pathSwitch_verbose_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BOLDBLUE()<<"path1_node ("<<path1_node<<"): "<<path1_node->getConfiguration().transpose()
               <<" -> path2_node ("<<path2_node<<"): "<<path2_node->getConfiguration().transpose()<<cnr_logger::RESET());

    std::vector<ConnectionPtr> path2_subpath_conn;
    /* Search for a better path2_subpath from path2_node */
    if(path2_node != goal_node_)
    {
      path2_subpath_conn = path2_subpath->getConnections();
      assert([&]() ->bool{
               path2_subpath->setMetrics(metrics_);
               if(path2_subpath->isValid())
               return true;
               else
               return false;
             }());

      if(full_net_search_)
      {
        double better_path2_subpath_cost;
        std::vector<ConnectionPtr> better_path2_subpath_conn;
        auto tic_map = graph_time::now();
        std::multimap<double,std::vector<ConnectionPtr>> path2_subpath_map = net_->getConnectionBetweenNodes(path2_node,goal_node_,path2_subpath_cost,{path1_node});

        double search_time = graph_duration(graph_time::now()-tic_map).count();

        tic_map = graph_time::now();
        if(findValidSolution(path2_subpath_map,path2_subpath_cost,better_path2_subpath_conn,better_path2_subpath_cost))
        {
          path2_subpath_conn.clear();
          path2_subpath_conn = better_path2_subpath_conn;
          path2_subpath = std::make_shared<Path>(path2_subpath_conn,metrics_,checker_,logger_);
          path2_subpath_cost = better_path2_subpath_cost;

          assert(better_path2_subpath_cost == path2_subpath->cost());

          if(pathSwitch_verbose_)
            CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"A better path2_subpath has been found!\n"<<*path2_subpath<<cnr_logger::RESET());
        }

        if(pathSwitch_verbose_)
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"Time to search for a better subpath2: "<<search_time<<", time to check solutions: "<<graph_duration(graph_time::now()-tic_map).count()<<cnr_logger::RESET());
      }
    }

    double diff_subpath_cost = candidate_solution_cost - path2_subpath_cost; // it is the maximum cost to make the connecting_path convenient
    double utopia = ps_goal->utopia; // utopia is the minimum cost that the connecting_path can have

    if(pathSwitch_disp_ || pathSwitch_verbose_)
    {
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"candidate_solution_cost: "<<candidate_solution_cost<<" subpath2_cost: "<<path2_subpath_cost<<cnr_logger::RESET());
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"diff_subpath_cost: "<< diff_subpath_cost<<" utopia: " << utopia<<cnr_logger::RESET());
    }

    /* The utopia between the two nodes must be less than
     * the maximum cost allowed for connecting_path */
    if(diff_subpath_cost>(utopia+1e-03))
    {
      at_least_a_trial_ = true;

      assert([&]() ->bool{
               if(path2_subpath != nullptr)
               {
                 std::vector<NodePtr> nodes_of_subpath2 = path2_subpath->getNodes();
                 if(std::find(nodes_of_subpath2.begin(),nodes_of_subpath2.end(),path1_node)<nodes_of_subpath2.end())
                 {
                   CNR_INFO(logger_,"path1_node: "<<*path1_node<<path1_node);
                   CNR_INFO(logger_,"path2 contains path1_node: \n"<<*path2_subpath);
                   CNR_INFO(logger_,"curr sol cost: %f, utopia: %f, diff_subpath_cost: %f, cost subpath2: %f",candidate_solution_cost,utopia,diff_subpath_cost,path2_subpath->cost());

             #ifdef ROS_AVAILABLE
                   disp_->displayNode(path1_node);
                   disp_->displayPath(path2_subpath);
             #endif

                   return false;
                 }
               }
               return true;
             }());

      PathPtr connecting_path;
      bool quickly_solved = false;

      auto tic_connecting_path = graph_time::now();
      bool connecting_path_found = computeConnectingPath(path1_node,path2_node,diff_subpath_cost,current_path,tic,tic_cycle,connecting_path,quickly_solved);

      if(pathSwitch_verbose_)
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"Time for computing connecting path "<<graph_duration(graph_time::now()-tic_connecting_path).count()<<" s"<<" max ps time "<<(pathSwitch_max_time_-graph_duration(graph_time::now()-tic).count())<<cnr_logger::RESET());

      if(connecting_path_found)
      {
        //if(not connecting_path->onLine())
        //{
        //  double opt_time = maxSolverTime(tic,tic_cycle);
        //  optimizePath(connecting_path,opt_time);
        //}

        assert(connecting_path->isValid());
        double new_solution_cost = path2_subpath_cost + connecting_path->cost();

        if(pathSwitch_verbose_ || pathSwitch_disp_)
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"solution cost: "<<new_solution_cost<<cnr_logger::RESET());

        if(new_solution_cost<candidate_solution_cost)
        {
          std::vector<ConnectionPtr> new_path_conn = connecting_path->getConnections();

          if(not path2_subpath_conn.empty())
            new_path_conn.insert(new_path_conn.end(),path2_subpath_conn.begin(),path2_subpath_conn.end());

          new_path = std::make_shared<Path>(new_path_conn, metrics_, checker_, logger_);
          new_path->setTree(tree_);
          assert(new_path->isValid());

          candidate_solution_cost = new_path->cost();

          path1_node_of_sol = path1_node;
          path2_node_of_sol = path2_node;

          success = true;
          an_obstacle_ = false;

#ifdef ROS_AVAILABLE
          if(pathSwitch_disp_)
          {
            disp_->clearMarker(pathSwitch_path_id_);
            disp_->changeConnectionSize(ps_marker_scale_);
            pathSwitch_path_id_ = disp_->displayPath(new_path,"pathplan",ps_marker_color_);
            disp_->defaultConnectionSize();
          }
#endif
        }
        else
        {
          if(pathSwitch_verbose_ || pathSwitch_disp_)
            CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"It is not a better solution"<<cnr_logger::RESET());

#ifdef ROS_AVAILABLE
          if(pathSwitch_disp_)
          {
            disp_->changeNodeSize(ps_marker_scale_sphere_);
            new_node_id = disp_->displayNode(path2_node,"pathplan",ps_marker_color_sphere_);
            disp_->defaultNodeSize();

            node_id_vector.push_back(new_node_id);
          }
#endif
        }

        toc_cycle = graph_time::now();
        if(pathSwitch_verbose_)
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BOLDYELLOW()<<"SOLVED->cycle time: "<<graph_duration(toc_cycle-tic_cycle).count()<<cnr_logger::RESET());

        if(not quickly_solved)  // not directly connected, usually it is very fast and it would alterate the mean value
        {
          time_vector.push_back(graph_duration(toc_cycle-tic_cycle).count());
          pathSwitch_cycle_time_mean_ = std::accumulate(time_vector.begin(), time_vector.end(),0.0)/((double) time_vector.size());

          if(pathSwitch_verbose_)
            CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"cycle time mean updated: "<<pathSwitch_cycle_time_mean_<<cnr_logger::RESET());
        }
        else
        {
          if(pathSwitch_verbose_)
            CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"cycle time mean not updated"<<cnr_logger::RESET());
        }

#ifdef ROS_AVAILABLE
        if(pathSwitch_disp_)
          disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
#endif
      }
      else
      {
        if((not an_obstacle_) && (pathSwitch_cycle_time_mean_ != std::numeric_limits<double>::infinity()))
        {
          pathSwitch_cycle_time_mean_ = 1.2*pathSwitch_cycle_time_mean_;

          if(pathSwitch_verbose_)
            CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"cycle time mean increased of 20%: "<<pathSwitch_cycle_time_mean_<<cnr_logger::RESET());
        }

#ifdef ROS_AVAILABLE
        if(pathSwitch_disp_)
        {
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"Not solved"<<cnr_logger::RESET());

          disp_->changeNodeSize(ps_marker_scale_sphere_);
          new_node_id = disp_->displayNode(path2_node,"pathplan",ps_marker_color_sphere_);
          disp_->defaultNodeSize();

          node_id_vector.push_back(new_node_id);

          disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
        }
#endif
      }
    }
    else
    {
      if(pathSwitch_verbose_ || pathSwitch_disp_)
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"It would not be a better solution"<<cnr_logger::RESET());

#ifdef ROS_AVAILABLE
      if(pathSwitch_disp_)
      {
        disp_->changeNodeSize(ps_marker_scale_sphere_);
        new_node_id = disp_->displayNode(path2_node,"pathplan",ps_marker_color_sphere_);
        disp_->defaultNodeSize();

        node_id_vector.push_back(new_node_id);

        disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
      }
#endif
    }

    if(pathSwitch_verbose_)
    {
      toc=graph_time::now();
      time = pathSwitch_max_time_ - graph_duration(toc-tic).count();
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"cycle time mean: "<<pathSwitch_cycle_time_mean_<<" -> available time: "<< time<<cnr_logger::RESET());
    }

    toc=graph_time::now();
    time = pathSwitch_max_time_ - graph_duration(toc-tic).count();
    if((!an_obstacle_ && time<time_percentage_variability_*pathSwitch_cycle_time_mean_ && pathSwitch_cycle_time_mean_ != std::numeric_limits<double>::infinity()) || time<=0.0)  //if there is an obstacle, you should use the entire available time to find a feasible solution
    {
      if(pathSwitch_verbose_)
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"TIME OUT! max time: "<<pathSwitch_max_time_<<", time_available: "<<time<<", time needed for a new cycle: "<<time_percentage_variability_*pathSwitch_cycle_time_mean_<<"; "<<remaining_goals<<" goals not considered."<<cnr_logger::RESET());

      break;
    }

    if(pathSwitch_verbose_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"PathSwitch cycle time: "<<graph_duration(graph_time::now()-tic_cycle).count()<<cnr_logger::RESET());
  }

  if(pathSwitch_verbose_ || pathSwitch_disp_)
  {
    if(pathSwitch_verbose_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"PathSwitch duration: "<<graph_duration(graph_time::now()-tic).count()<<cnr_logger::RESET());

#ifdef ROS_AVAILABLE
    if(pathSwitch_disp_)
    {
      for(const int& id_to_delete:node_id_vector)
        disp_->clearMarker(id_to_delete);
    }
#endif

    if(success)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"PathSwitch has found a solution with cost: " << new_path->cost()<<". path1_node ("<<path1_node_of_sol<<"): "<<path1_node_of_sol->getConfiguration().transpose()<<" -> path2_node ("<<path2_node_of_sol<<"): "<<path2_node_of_sol->getConfiguration().transpose()<<cnr_logger::RESET());
    else
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BLUE()<<"PathSwitch has NOT found a solution"<<cnr_logger::RESET());
  }

  return success;
}

PathPtr MARS::getSubpath1(NodePtr& current_node)
{
  /* If the current configuration matches a node of the current_path_ */
  current_node = nullptr;
  std::vector<NodePtr> current_path_nodes = current_path_->getNodes();

  if((current_configuration_-current_path_nodes.back()->getConfiguration()).norm()<=TOLERANCE)
  {
    if(informedOnlineReplanning_verbose_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"The current node is the goal!"<<cnr_logger::RESET());

    current_node = current_path_nodes.back();

    return nullptr;
  }

  for(unsigned int i=0;i<current_path_nodes.size()-1;i++)
  {
    if((current_configuration_-current_path_nodes.at(i)->getConfiguration()).norm()<=TOLERANCE)
    {
      current_node = current_path_nodes.at(i);
      PathPtr subpath1 = current_path_->getSubpathFromNode(current_node);
      subpath1->setTree(tree_);

      return subpath1;
    }
  }

  CNR_ERROR(logger_,"Current configuration ("<<current_configuration_.transpose()<<") does not match any node of the current path!");
  CNR_ERROR(logger_,"Current path: "<<*current_path_);
  assert(0);

  return nullptr;
}

void MARS::initFlaggedConnections()
{
  clearFlaggedConnections();

  flagged_connections_ = current_path_->getConnections();
  for(const PathPtr& p:other_paths_)
    flagged_connections_.insert(flagged_connections_.end(),p->getConnectionsConst().begin(),p->getConnectionsConst().end());

  std::for_each(flagged_connections_.begin(),flagged_connections_.end(),
                [&](ConnectionPtr& checked_conn) ->void{
    checked_conn->setRecentlyChecked(true);
  });
}

void MARS::clearFlaggedConnections()
{
  std::for_each(flagged_connections_.begin(),flagged_connections_.end(),
                [&](ConnectionPtr& checked_conn) ->void{
    checked_conn->setRecentlyChecked(false);
  });

  flagged_connections_.clear();
}

bool MARS::informedOnlineReplanning(const double &max_time)
{
  auto tic=graph_time::now();
  graph_time_point toc, tic_cycle, toc_cycle;

  double MAX_TIME;
  if(informedOnlineReplanning_disp_)
    MAX_TIME = std::numeric_limits<double>::infinity();
  else
    MAX_TIME = max_time;

  available_time_ = MAX_TIME;
  const double TIME_LIMIT = 0.85*MAX_TIME; //seconds
  const int CONT_LIMIT = 5;

  if(not informedOnlineReplanning_disp_ && available_time_<=0.0)
    return false;

  std::vector<PathPtr> reset_other_paths;
  std::vector<NodePtr> examined_nodes;
  PathPtr new_path, replanned_path;
  PathPtr admissible_current_path = nullptr;
  bool exit = false;
  bool solved = false;
  bool first_sol = true;
  unsigned int cont = 0;
  double previous_cost;
  double replanned_path_cost = std::numeric_limits<double>::infinity();

  success_ = false;
  an_obstacle_ = false;
  at_least_a_trial_ = false;

#ifdef ROS_AVAILABLE
  int replanned_path_id;
#endif

  /* Set the connections of the available paths to recently checked, they don't need a collision check
   * by the replanner because they are checked externally */
  initFlaggedConnections();

  /* Add the valid portion of the current path to the set of available paths */
  size_t current_conn_idx;
  current_path_->findConnection(current_configuration_,current_conn_idx);

  admissible_other_paths_.clear();
  reset_other_paths = addAdmissibleCurrentPath(current_conn_idx,admissible_current_path);
  admissible_other_paths_ = reset_other_paths;

  /* Compute subpath1 */
  NodePtr current_node;
  PathPtr subpath1 = getSubpath1(current_node); //nullptr if subpath1 does not exist (current_node = goal)
  if(subpath1)
  {
    if(subpath1->cost() == std::numeric_limits<double>::infinity())
      an_obstacle_ = true;
  }
  else
  {
    if(informedOnlineReplanning_verbose_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"The current configuration matches with the goal OR does not match with any node of the current path!"<<cnr_logger::RESET());

    /* Clear flagged connections vector */
    clearFlaggedConnections();

    success_ = false;
    return false;
  }

  /* Searching for an already existing solution */
  if(informedOnlineReplanning_verbose_)
    CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"Searching for an already existing solution.."<<cnr_logger::RESET());

  full_net_search_?
        (replanned_path = bestExistingSolution(subpath1)): // if a solution is not found, replanned_path = subpath1
        (replanned_path = subpath1);

  replanned_path_cost = replanned_path->cost();

  assert(replanned_path->getStartNode() == current_node);

  if(replanned_path != subpath1)  // if an already existing solution has been found (different from subpath1), success = true
  {
    success_ = true;
    assert(replanned_path->cost()<std::numeric_limits<double>::infinity());

    if(informedOnlineReplanning_verbose_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"A ready-to-use solution has been found, cost: "<<replanned_path_cost<<cnr_logger::RESET());
  }
  else
  {
    if(informedOnlineReplanning_verbose_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"A solution better than subpath1 has not been found, cost: "<<replanned_path_cost<<cnr_logger::RESET());
  }

  assert(replanned_path->getTree() == tree_);

#ifdef ROS_AVAILABLE
  if(informedOnlineReplanning_disp_)
  {
    disp_->changeConnectionSize(informed_marker_scale_);
    replanned_path_id = disp_->displayPath(replanned_path,"pathplan",informed_marker_color_);
    disp_->defaultConnectionSize();
    disp_->nextButton("Press Next to start searching for a better solution");
  }
#endif

  std::vector<NodePtr> start_node_vector = startNodes(replanned_path->getConnectionsConst());

  int j = start_node_vector.size()-1;
  NodePtr start_node_for_pathSwitch;

  while(j>=0)
  {
    tic_cycle = graph_time::now();

    start_node_for_pathSwitch = start_node_vector.at(j);
    assert(start_node_for_pathSwitch->getParentConnectionsSize() == 1);
    simplifyAdmissibleOtherPaths(replanned_path,start_node_for_pathSwitch,reset_other_paths);

    if(informedOnlineReplanning_verbose_)
    {
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BOLDGREEN()<<"Starting nodes for replanning:"<<cnr_logger::RESET());
      for(const NodePtr& n:start_node_vector)
      {
        if(n != start_node_for_pathSwitch)
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BOLDGREEN()<<"n: "<<n->getConfiguration().transpose()
                   <<" ("<<n<<") examined: "<<n->getFlag(examined_flag_,false)<<cnr_logger::RESET());
        else
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BOLDGREEN()<<"n: "<<n->getConfiguration().transpose()
                   <<" ("<<n<<") examined: "<<n->getFlag(examined_flag_,false)<<" <--"<<cnr_logger::RESET());
      }

      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"current best solution path: "<<*replanned_path<<cnr_logger::RESET());
    }

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"j: "<<j<<cnr_logger::RESET());

    assert([&]() ->bool{
             for(const NodePtr& n:start_node_vector)
             {
               std::vector<NodePtr> nodes_rep = replanned_path->getNodes();
               if(std::find(nodes_rep.begin(),nodes_rep.end(),n)>=nodes_rep.end())
               {
                 CNR_INFO(logger_,"Node not found: "<<n->getConfiguration().transpose()<<" "<<n);

                 for(const NodePtr& nn:nodes_rep)
                 {
                   CNR_INFO(logger_,"n: "<<nn->getConfiguration().transpose()<<" "<<nn);
                 }
                 return false;
               }
             }
             return true;
           }());

#ifdef ROS_AVAILABLE
    if(informedOnlineReplanning_disp_)
    {
      disp_->changeNodeSize(informed_marker_scale_sphere_);
      disp_->displayNode(start_node_for_pathSwitch,"pathplan",informed_marker_color_sphere_);
      disp_->defaultNodeSize();
    }
#endif

    if(pathSwitch_cycle_time_mean_ >= 0.8*max_time)
      pathSwitch_cycle_time_mean_ = std::numeric_limits<double>::infinity();  //reset

    toc = graph_time::now();
    available_time_ = MAX_TIME - graph_duration(toc-tic).count();

    double min_time_to_launch_pathSwitch;
    if(informedOnlineReplanning_disp_)
      min_time_to_launch_pathSwitch = std::numeric_limits<double>::infinity();
    else if(an_obstacle_ || pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity())
      min_time_to_launch_pathSwitch = 0.0;
    else
      min_time_to_launch_pathSwitch = time_percentage_variability_*pathSwitch_cycle_time_mean_;

    if(informedOnlineReplanning_verbose_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"available time: "<<available_time_<<", min required time to call PathSwitch: "<<min_time_to_launch_pathSwitch<<cnr_logger::RESET());

    if(available_time_>=min_time_to_launch_pathSwitch)
    {
      if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"Launching PathSwitch..."<<cnr_logger::RESET());

      solved = pathSwitch(replanned_path,start_node_for_pathSwitch,new_path);

      start_node_vector.pop_back();

      start_node_for_pathSwitch->setFlag(examined_flag_,true);
      examined_nodes.push_back(start_node_for_pathSwitch);

      assert((solved && new_path->getTree() != nullptr) || (not solved));
      assert([&]() ->bool{
               if(std::find(start_node_vector.begin(),start_node_vector.end(),start_node_for_pathSwitch)>=start_node_vector.end())
               {
                 return true;
               }
               else
               {
                 for(const NodePtr& n:start_node_vector)
                 {
                   if(n == start_node_for_pathSwitch)
                   CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BOLDYELLOW()<<n->getConfiguration().transpose()<<" ("<<n<<")"<<cnr_logger::RESET());
                   else
                   CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BOLDYELLOW()<<n->getConfiguration().transpose()<<" ("<<n<<")"<<cnr_logger::RESET());
                 }
                 return false;
               }
             }());
    }
    else
    {
      exit = true;
      solved = false;

      if(informedOnlineReplanning_verbose_)
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"Not enough time to call PathSwitch, available time: "<<available_time_<<" min time to call PathSwitch: "<<min_time_to_launch_pathSwitch<<cnr_logger::RESET());
    }

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"Solved: "<<solved<<cnr_logger::RESET());

    if(solved)
    {
      std::vector<ConnectionPtr> candidate_solution_conn;
      PathPtr subpath_to_start_node_for_pathSwitch,candidate_solution;

      if(start_node_for_pathSwitch != current_node)
      {
        subpath_to_start_node_for_pathSwitch = replanned_path->getSubpathToNode(start_node_for_pathSwitch);
        assert(subpath_to_start_node_for_pathSwitch->isValid());

        assert(replanned_path->getStartNode() == current_node);

        if(full_net_search_)
        {
          double subpath_to_start_node_for_pathSwitch_cost;
          std::multimap<double,std::vector<ConnectionPtr>> map_to_start_node_for_pathSwitch = net_->getConnectionBetweenNodes(current_node,start_node_for_pathSwitch,subpath_to_start_node_for_pathSwitch->cost(),{});

          if(findValidSolution(map_to_start_node_for_pathSwitch,subpath_to_start_node_for_pathSwitch->cost(),
                               candidate_solution_conn,subpath_to_start_node_for_pathSwitch_cost))
          {
            if(not reverse_start_nodes_)
            {
              start_node_vector.clear();
              start_node_vector = startNodes(candidate_solution_conn); //if a solution different from subpath_to_start_node_for_pathSwitch is found, update the nodes
              j = start_node_vector.size();
            }
          }
          else
          {
            candidate_solution_conn = subpath_to_start_node_for_pathSwitch->getConnections();
          }
        }
        else
        {
          candidate_solution_conn = subpath_to_start_node_for_pathSwitch->getConnections();
        }

        candidate_solution_conn.insert(candidate_solution_conn.end(),new_path->getConnectionsConst().begin(),new_path->getConnectionsConst().end());
        candidate_solution = std::make_shared<Path>(candidate_solution_conn,metrics_,checker_,logger_);
        candidate_solution->setTree(tree_);
      }
      else
      {
        candidate_solution = new_path;
      }

      assert((candidate_solution->getTree() == new_path->getTree()) && (candidate_solution->getTree() == tree_));

      if(candidate_solution->cost()<replanned_path_cost)
      {
        if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"new path found, cost: " << candidate_solution->cost() <<" previous cost: " << replanned_path_cost<<cnr_logger::RESET());

        if(first_sol)
        {
          toc = graph_time::now();
          time_first_sol_ = graph_duration(toc - tic).count();
          time_replanning_ = time_first_sol_;
          first_sol = false;
        }

        previous_cost = replanned_path_cost;
        replanned_path = candidate_solution;
        replanned_path_cost = candidate_solution->cost();

        assert(replanned_path->getTree() == tree_);
        assert(replanned_path_cost < std::numeric_limits<double>::infinity());
        assert(replanned_path->getStartNode() == current_node);

        if(reverse_start_nodes_)
        {
          start_node_vector.clear();
          start_node_vector = startNodes(replanned_path->getConnections());
          j = start_node_vector.size();
        }

        success_ = true;
        an_obstacle_ = false;

#ifdef ROS_AVAILABLE
        if(informedOnlineReplanning_disp_)
        {
          disp_->clearMarker(pathSwitch_path_id_);
          disp_->clearMarker(replanned_path_id);
          disp_->changeConnectionSize(informed_marker_scale_);
          replanned_path_id = disp_->displayPath(replanned_path,"pathplan",informed_marker_color_);
          disp_->defaultConnectionSize();
        }
#endif

        toc = graph_time::now();
        if(graph_duration(toc-tic).count()>TIME_LIMIT && cont >= CONT_LIMIT)
        {
          j = -1;
          break;
        }
        else
          ((previous_cost-replanned_path_cost)<(0.05*previous_cost))? cont++:
                                                                      cont = 0;
      }
      else
      {
        if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"NO better path found, cost: " << candidate_solution->cost() <<" previous cost: " << replanned_path_cost<<cnr_logger::RESET());
      }

      toc_cycle = graph_time::now();
      if(informedOnlineReplanning_verbose_)
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"Solution with cost "<<replanned_path_cost<<" found!->Informed cycle duration: "<<graph_duration(toc_cycle-tic_cycle).count()<<"\n"<<*replanned_path<<cnr_logger::RESET());
    }

    toc = graph_time::now();
    available_time_ = MAX_TIME-graph_duration(toc-tic).count();

    if(j == 0 && (available_time_>=0))
    {
      if(success_)
      {
        if(replanned_path->getConnectionsSize()>1)
        {
          if(full_net_search_)
          {
            std::multimap<double,std::vector<ConnectionPtr>> best_replanned_path_map  = net_->getConnectionBetweenNodes(current_node,goal_node_,replanned_path->cost());

            double best_replanned_path_cost;
            std::vector<ConnectionPtr> best_replanned_path_conns;
            if(findValidSolution(best_replanned_path_map,replanned_path->cost(),best_replanned_path_conns,best_replanned_path_cost))
            {
              replanned_path = std::make_shared<Path>(best_replanned_path_conns,metrics_,checker_,logger_);
              replanned_path->setTree(tree_);
            }
          }

          if(reverse_start_nodes_)
          {
            start_node_vector.clear();
            start_node_vector = startNodes(replanned_path->getConnectionsConst());
            j = start_node_vector.size();
          }
        }
      }
      else
      {
        if(not current_node->getFlag(examined_flag_,false))
        {
          if(not replanned_path->onLine())
          {
            if(not at_least_a_trial_)
            {
              at_least_a_trial_ = true;

              start_node_vector.clear();
              start_node_vector.push_back(current_node);
              j = start_node_vector.size();
            }
          }
        }
      }
    }

    toc = graph_time::now();
    available_time_ = MAX_TIME-graph_duration(toc-tic).count();

    if(exit || j==0)
    {
      if(informedOnlineReplanning_verbose_ && exit)
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"TIME OUT! available time: "<<available_time_<<", time needed for a new cycle: "<<min_time_to_launch_pathSwitch<<cnr_logger::RESET());

#ifdef ROS_AVAILABLE
      if(informedOnlineReplanning_disp_)
      {
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"Optimizing..."<<cnr_logger::RESET());
        disp_->nextButton();
      }
#endif
      j = -1;
      break;
    }

    j -= 1;

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"------------------------------------------"<<cnr_logger::RESET());

#ifdef ROS_AVAILABLE
    if(informedOnlineReplanning_disp_)
      disp_->nextButton("Press \"next\" to execute the next InformedOnlineReplanning step");
#endif

  } //end while(j>=0) cycle

  std::for_each(examined_nodes.begin(),examined_nodes.end(),
                [&](NodePtr& examined_node) ->void{examined_node->setFlag(examined_flag_,false);});

  if(success_)
  {
    assert(std::abs(replanned_path_cost - replanned_path->cost())<1e-06);

    double net_search_time;
    available_time_ = MAX_TIME-graph_duration(graph_time::now()-tic).count();

    if(available_time_<=0.0)
      net_search_time = 0.0;
    else if(available_time_<0.02 && available_time_>0.0)
      net_search_time = 0.02; //max 20 milliseconds
    else
      net_search_time = available_time_;

    if(informedOnlineReplanning_verbose_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"Time before net search: "<<available_time_<<", max net time: "<<net_search_time<<cnr_logger::RESET());

    auto tic_net_search = graph_time::now();
    std::multimap<double,std::vector<ConnectionPtr>> best_replanned_path_map  =
        net_->getConnectionBetweenNodes(current_node,goal_node_,replanned_path->cost(),{},net_search_time*0.8);
    auto toc_net_search = graph_time::now();
    if(graph_duration(toc_net_search-tic_net_search).count()>net_search_time/0.5 && net_search_time>0.0)
      throw std::runtime_error("net too much time: "+std::to_string(graph_duration(toc_net_search-tic_net_search).count())+ " max time "+std::to_string(net_search_time));

    double best_replanned_path_cost;
    std::vector<ConnectionPtr> best_replanned_path_conns;
    auto tic_find_sol = graph_time::now();
    if(findValidSolution(best_replanned_path_map,replanned_path->cost(),best_replanned_path_conns,best_replanned_path_cost))
    {
      replanned_path = std::make_shared<Path>(best_replanned_path_conns,metrics_,checker_,logger_);
      replanned_path->setTree(tree_);

      if(informedOnlineReplanning_verbose_)
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"A better path exists! Cost: "<<replanned_path->cost()<<" previous cost: "<<replanned_path_cost<<cnr_logger::RESET());

      replanned_path_cost = replanned_path->cost();
    }
    auto toc_find_sol = graph_time::now();
    assert(replanned_path->cost()<std::numeric_limits<double>::infinity());
    assert(replanned_path->cost()<=subpath1->cost());

    if(informedOnlineReplanning_verbose_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"At the end of replanning, in the graph there are "<<best_replanned_path_map.size()<<" paths better the one found! (found in "<<graph_duration(toc_net_search-tic_net_search).count()
               <<" s), time to check solutions "<<graph_duration(toc_find_sol-tic_find_sol).count()<<" s"<<cnr_logger::RESET());

    replanned_path_ = replanned_path;
    assert(replanned_path_->isValid());
    assert(replanned_path_->getTree() == tree_);

    toc = graph_time::now();
    time_replanning_ = graph_duration(toc - tic).count();

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
    {
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"InformedOnlineReplanning has found a solution with cost: " <<replanned_path_->cost() << " in "<< time_replanning_ << "seconds."<<cnr_logger::RESET());
    }
  }
  else
  {
    success_ = false;
    replanned_path_ = subpath1;

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_)
      CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"InformedOnlineReplanning has NOT found a solution"<<cnr_logger::RESET());
  }

  assert([&]() ->bool{
           for(const ConnectionPtr& c:replanned_path_->getConnectionsConst())
           {
             if(not c->isRecentlyChecked())
             {
               CNR_ERROR(logger_,"replanned path "<<*replanned_path_);
               return false;
             }
           }
           return true;
         }());

  /* Clear the vector of connections set invalid during the replanner call */
  clearInvalidConnections();
  assert(invalid_connections_.empty());

  /* Clear flagged connections vector -> after clearInvalidConnections */
  clearFlaggedConnections();
  assert(flagged_connections_.empty());

  toc = graph_time::now();
  available_time_ = MAX_TIME-graph_duration(toc-tic).count();

  return success_;
}

bool MARS::replan()
{
  auto tic = graph_time::now();
  success_ = false;

  if(not checker_->check(current_configuration_))
  {
    CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::RED()<<"current replan configuration in collision!"<<cnr_logger::RESET());
    success_ = false;
    is_a_new_node_ = false;

    return false;
  }

  size_t conn_idx;
  ConnectionPtr conn = current_path_->findConnection(current_configuration_,conn_idx);
  NodePtr current_node = current_path_->addNodeAtCurrentConfig(current_configuration_,conn,true,is_a_new_node_);

  std::vector<unsigned int> sizes;
  for(const PathPtr& p:other_paths_)
    sizes.push_back(p->getConnectionsSize());

  /* If a new node is added, add it also to the other path if conn is also a connection of one of the other paths */
  std::vector<PathPtr> other_paths_changed;
  if(is_a_new_node_)
  {
    for(PathPtr &p:other_paths_)
    {
      if(p->splitConnection(current_path_->getConnectionsConst().at(conn_idx),
                            current_path_->getConnectionsConst().at(conn_idx+1),conn))
        other_paths_changed.push_back(p);
    }
  }

  double current_cost = current_path_->getCostFromConf(current_configuration_);

  if(verbose_)
  {
    CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"Starting node for replanning: \n"<< *current_node<<current_node<<"\nis a new node: "<<is_a_new_node_<<cnr_logger::RESET());
    CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::GREEN()<<"Cost from here: "<<current_cost<<cnr_logger::RESET());
  }


  //  assert([&]() ->bool{
  //           double cost = current_path_->cost();
  //           current_path_->setChecker(checker_);
  //           current_path_->isValid();
  //           if(std::abs(cost - current_path_->cost())>1e-04)
  //           {
  //             CNR_INFO(logger_,"cost "<<cost<<" current cost "<<current_path_->cost());
  //             return false;
  //           }
  //           else
  //           return true;
  //         }());

  double max_time = max_time_-graph_duration(tic-graph_time::now()).count();
  success_ = informedOnlineReplanning(max_time);

  assert([&]() ->bool{
           if(not success_)
           {
             return true;
           }
           else
           {
             if(replanned_path_->isValid())
             return true;
             else
             return false;
           }
         }());

  bool path_changed = false;
  if(success_)
  {
    path_changed = true;
  }
  else
  {
    if(is_a_new_node_)
    {
      if(not current_path_->removeNode(current_node,{}))
      {
        path_changed = true;
      }
      else
      {
        assert(conn->getParent()->getConfiguration() == current_path_->getConnections().at(conn_idx)->getParent()->getConfiguration());
        assert(conn->getChild ()->getConfiguration() == current_path_->getConnections().at(conn_idx)->getChild ()->getConfiguration());

        for(PathPtr& p:other_paths_changed)
        {
          if(not p->restoreConnection(current_path_->getConnections().at(conn_idx),current_node))
            assert(0);
        }
      }
    }
  }

  assert([&]() ->bool{
           if(not path_changed)
           {
             bool equal_size;
             for(unsigned int i=0;i<other_paths_.size();i++)
             {
               equal_size = (sizes[i] == other_paths_[i]->getConnectionsSize());
               if(not equal_size)
               {
                 for(unsigned int ii=0;ii<other_paths_.size();ii++)
                 {
                   CNR_INFO(logger_,"p: "<<*other_paths_[ii]);
                   CNR_INFO(logger_,"old size: "<<sizes[ii]<<" size: "<<other_paths_[ii]->getConnectionsSize());
                   CNR_INFO(logger_,"------------------");
                 }
                 return false;
               }
             }
             return true;
           }
           else
           {
             if(replanned_path_->getNodes().front() != current_node)
             {
               CNR_INFO(logger_,"replanned path first node:\n"<<*replanned_path_->getNodes().front()<<replanned_path_->getNodes().front());
               CNR_INFO(logger_,"replan node:\n"<<*current_node<<current_node);
               CNR_INFO(logger_,"current path:\n"<<*current_path_);
               CNR_INFO(logger_,"replanned path:\n"<<*replanned_path_);
               CNR_INFO(logger_,"success: "<<success_);
               return false;
             }
             return true;
           }
         }());


  assert([&]() ->bool{
           if(path_changed)
           {
             for(const NodePtr& n:replanned_path_->getNodes())
             {
               if(n->getParentConnectionsSize()!=1)
               {
                 for(const NodePtr& nn:replanned_path_->getNodes())
                 {
                   CNR_INFO(logger_,nn<<" "<<*nn);
                 }
                 CNR_INFO(logger_,*replanned_path_);

                 return false;
               }
             }
           }
           return true;
         }());

  assert([&]() ->bool{

           for(const NodePtr& n:current_path_->getNodes())
           {
             if(n->getParentConnectionsSize()!=1)
             {
               for(const NodePtr& nn:current_path_->getNodes())
               {
                 CNR_INFO(logger_,nn<<" "<<*nn);
               }
               CNR_INFO(logger_,*current_path_);

               return false;
             }
           }

           return true;
         }());

  return path_changed;
}
}
