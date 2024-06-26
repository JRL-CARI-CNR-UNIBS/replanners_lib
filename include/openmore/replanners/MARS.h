#pragma once

#include <openmore/replanners/replanner_base.h>
#include <graph_core/graph/net.h>

/**
 * @file MARS.h
 * @brief From the paper Anytime Informed Multi-Path Replanning Strategy for Complex Environments (https://ieeexplore.ieee.org/abstract/document/10013661).
 */

namespace openmore
{

#define TIME_PERCENTAGE_VARIABILITY 0.7

struct ps_goal
{
  NodePtr node;
  double utopia;
  PathPtr subpath;
  double subpath_cost;
};
typedef std::shared_ptr<ps_goal> ps_goal_ptr;


struct invalid_connection
{
  ConnectionPtr connection;
  double cost;
};
typedef std::shared_ptr<invalid_connection> invalid_connection_ptr;


/**
 * @class MARS
 * @brief Class for multi-path replanning in high-dimensional search spaces.
 *
 * The algorithm replans by exploiting a set of pre-computed paths from the same start to the same goal,
 * and improves the solution over time. It searches for paths connecting the nodes of the current path
 * to the nodes of other available paths using subtrees, informed sets, and lazy collision checking.
 * Once a connecting path is found, the solution is obtained by concatenating the subpath from the
 * destination node to the goal. This procedure is continuously repeated to find better solutions.
 */
class MARS;
typedef std::shared_ptr<MARS> MARSPtr;

class MARS: public ReplannerBase
{
protected:

  NetPtr net_;
  TreePtr tree_;
  NodePtr paths_start_;
  std::vector<PathPtr> other_paths_;
  std::vector<PathPtr> admissible_other_paths_;
  std::vector<ConnectionPtr> flagged_connections_;
  std::vector<invalid_connection_ptr> invalid_connections_;

  double time_first_sol_;
  double time_replanning_;
  double available_time_;
  double pathSwitch_max_time_;
  double pathSwitch_cycle_time_mean_;
  double time_percentage_variability_;

  int pathSwitch_path_id_;
  unsigned int examined_flag_; // used to store which nodes has been already examined

  bool an_obstacle_;
  bool is_a_new_node_;
  bool full_net_search_;
  bool at_least_a_trial_;
  bool pathSwitch_disp_;
  bool pathSwitch_verbose_;
  bool reverse_start_nodes_;
  bool informedOnlineReplanning_disp_;
  bool informedOnlineReplanning_verbose_;

  std::vector<double> ps_marker_scale_              = {0.01,0.01,0.01   };
  std::vector<double> ps_marker_color_              = {1.0,0.5,0.0,1.0  };
  std::vector<double> informed_marker_scale_        = {0.01,0.01,0.01   };
  std::vector<double> informed_marker_color_        = {1.0,1.0,0.0,1.0  };
  std::vector<double> ps_marker_color_sphere_       = {0.5,0.5,0.5,1.0  };
  std::vector<double> ps_marker_scale_sphere_       = {0.025,0.025,0.025};
  std::vector<double> informed_marker_scale_sphere_ = {0.03,0.03,0.03   };
  std::vector<double> informed_marker_color_sphere_ = {1.0,0.5,0.0,1.0  };

  std::vector<PathPtr> addAdmissibleCurrentPath(const size_t &idx_current_conn, PathPtr& admissible_current_path);
  PathPtr getSubpath1(NodePtr& current_node);
  PathPtr bestExistingSolution(const PathPtr& current_solution);
  PathPtr bestExistingSolution(const PathPtr& current_solution, std::multimap<double, std::vector<ConnectionPtr> > &tmp_map);
  double maxSolverTime(const graph_time_point& tic, const graph_time_point& tic_cycle);
  void simplifyAdmissibleOtherPaths(const PathPtr& current_solution_path, const NodePtr &start_node, const std::vector<PathPtr>& reset_other_paths);
  bool mergePathToTree(const PathPtr &path);
  void convertToSubtreeSolution(const PathPtr& net_solution, const std::vector<NodePtr>& black_nodes);

  bool findValidSolution(const std::multimap<double,std::vector<ConnectionPtr>> &map, const double& cost2beat, std::vector<ConnectionPtr>& solution, double &cost, bool verbose = false);
  virtual bool findValidSolution(const std::multimap<double,std::vector<ConnectionPtr>> &map, const double& cost2beat, std::vector<ConnectionPtr>& solution, double &cost, unsigned int &number_of_candidates, bool verbose = false);

  virtual void initFlaggedConnections();
  virtual void clearInvalidConnections();
  virtual void clearFlaggedConnections();
  virtual std::vector<ps_goal_ptr> sortNodes(const NodePtr& node);
  virtual std::vector<NodePtr> startNodes(const std::vector<ConnectionPtr>& subpath1_conn);
  virtual bool computeConnectingPath(const NodePtr &path1_node_fake, const NodePtr &path2_node, const double &diff_subpath_cost, const PathPtr &current_solution, const graph_time_point &tic, const graph_time_point &tic_cycle, PathPtr &connecting_path, bool &quickly_solved);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MARS(const Eigen::VectorXd& current_configuration,
       const PathPtr& current_path,
       const double& max_time,
       const TreeSolverPtr &solver, const cnr_logger::TraceLoggerPtr &logger);

  MARS(const Eigen::VectorXd& current_configuration,
       const PathPtr& current_path,
       const double& max_time,
       const TreeSolverPtr &solver, const cnr_logger::TraceLoggerPtr &logger,
       const std::vector<PathPtr> &other_paths);

  NetPtr getNet()
  {
    return net_;
  }

  std::vector<PathPtr> getOtherPaths()
  {
    return other_paths_;
  }

  void setVerbosity(const bool& verbose) override
  {
    verbose_ = verbose;
    informedOnlineReplanning_verbose_ = verbose;
    pathSwitch_verbose_ = verbose;
  }

  void setVerbosityLevel(const int& verbose)
  {
    switch(verbose)
    {
    case 0:
      verbose_ = false;
      pathSwitch_verbose_ = false;
      informedOnlineReplanning_verbose_ = false;
      break;
    case 1:
      verbose_ = true;
      pathSwitch_verbose_ = false;
      informedOnlineReplanning_verbose_ = true;
      break;
    case 2:
      verbose_ = true;
      pathSwitch_verbose_ = true;
      informedOnlineReplanning_verbose_ = true;
      break;
    default:
      CNR_ERROR(logger_,"Verbosity level should be <= 2, set equal to 2");
      verbose_ = true;
      pathSwitch_verbose_ = true;
      informedOnlineReplanning_verbose_ = true;
    }
  }

  void setFullNetSearch(const bool full_net_search)
  {
    full_net_search_ = full_net_search;
  }

  void reverseStartNodes(const bool reverse)
  {
    reverse_start_nodes_ = reverse;
  }

  void setInformedOnlineReplanningVerbose(const bool verbose)
  {
    informedOnlineReplanning_verbose_ = verbose;
  }

  void setPathSwitchVerbose(const bool verbose)
  {
    pathSwitch_verbose_ = verbose;
  }

  void setInformedOnlineReplanningDisp(const bool verbose)
  {
#ifdef GRAPH_DISPLAY_AVAILABLE
    informedOnlineReplanning_disp_ = verbose;
#else
    informedOnlineReplanning_disp_ = false;
    CNR_WARN(logger_,"ROS is not available, cannot use graph_display");
#endif
  }

  void setPathSwitchDisp(const bool verbose)
  {
#ifdef GRAPH_DISPLAY_AVAILABLE
    pathSwitch_disp_ = verbose;
#else
    pathSwitch_disp_ = false;
    CNR_WARN(logger_,"ROS is not available, cannot use graph_display");
#endif
  }

  virtual void setCurrentPath(const PathPtr& path) override
  {
    current_path_ = path;

    bool is_a_new_tree = (tree_ != current_path_->getTree());
    tree_ = current_path_->getTree();
    if(is_a_new_tree)
      copyTreeRoot();

    net_->setTree(tree_);
    admissible_other_paths_ = other_paths_;
    goal_node_ = current_path_->getConnections().back()->getChild();
    success_ = false;
  }

  void copyTreeRoot();

  virtual void setOtherPaths(const std::vector<PathPtr> &other_paths, const bool merge_tree = true)
  {
    other_paths_.clear();

    for(const PathPtr& path:other_paths)
      addOtherPath(path,merge_tree);

    admissible_other_paths_ = other_paths_;
    success_ = false;
  }

  void setCurrentConf(const Eigen::VectorXd& q) override
  {
    current_configuration_ = q;
    success_ = false;
  }

  void setChecker(const CollisionCheckerPtr& checker) override
  {
    ReplannerBase::setChecker(checker);
    for(const PathPtr& p:other_paths_)
      p->setChecker(checker);
  }

  virtual void addOtherPath(const PathPtr& path, bool merge_tree = true)
  {
    if(merge_tree)
    {
      if(not mergePathToTree(path))
        assert(0);
    }

    other_paths_.push_back(path);
  }

  ReplannerBasePtr pointer()
  {
    return shared_from_this();
  }

  virtual bool pathSwitch(const PathPtr& current_path, const NodePtr& path1_node, PathPtr &new_path);
  virtual bool informedOnlineReplanning(const double &max_time  = std::numeric_limits<double>::infinity());

  virtual bool replan() override;
};
}

