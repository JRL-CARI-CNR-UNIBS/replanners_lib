#pragma once

#include <replanners_lib/replanners/replanner_base.h>
#include <graph_core/solvers/rrt_star.h>

//Dynamic path planning and replanning for mobile robots using RRT*

namespace openmore
{
class DynamicRRTStar;
typedef std::shared_ptr<DynamicRRTStar> DynamicRRTStarPtr;

class DynamicRRTStar: public ReplannerBase
{
protected:

  bool nodeBeforeObs(const PathPtr& subpath, NodePtr& node_before);
  bool nodeBehindObs(NodePtr& node_behind);
  bool connectBehindObs(const NodePtr &node);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DynamicRRTStar(Eigen::VectorXd& current_configuration,
                 PathPtr& current_path,
                 const double& max_time,
                 const TreeSolverPtr &solver,
                 const cnr_logger::TraceLoggerPtr &logger);

  bool replan() override;
};
}
