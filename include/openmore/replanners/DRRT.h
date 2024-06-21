#pragma once

#include <openmore/replanners/replanner_base.h>
#include <graph_core/solvers/rrt.h>
#include <typeinfo>

//Replanning with RRTs

namespace openmore
{
class DynamicRRT;
typedef std::shared_ptr<DynamicRRT> DynamicRRTPtr;

class DynamicRRT: public ReplannerBase
{
protected:
  NodePtr node_replan_;
  TreePtr trimmed_tree_;
  bool tree_is_trimmed_;
  InformedSamplerPtr sampler_;
  std::vector<ConnectionPtr> checked_connections_;

  virtual bool trimInvalidTree(NodePtr& node);
  virtual bool regrowRRT(NodePtr& node);
  bool replan(const double& cost_from_conf);
  void fixTree(const NodePtr& node_replan, const NodePtr& root, std::vector<NodePtr> &old_nodes, std::vector<double> &old_connections_costs);
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DynamicRRT(Eigen::VectorXd& current_configuration,
             PathPtr& current_path,
             const double& max_time,
             const TreeSolverPtr &solver,
             const cnr_logger::TraceLoggerPtr &logger);

  bool getTreeIsTrimmed()
  {
    return tree_is_trimmed_;
  }

  virtual bool replan() override;
};
}
