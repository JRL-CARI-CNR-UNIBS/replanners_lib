#pragma once

#include <openmore/replanners/DRRT.h>
#include <graph_core/solvers/anytime_rrt.h>

//Anytime, Dynamic Planning in High-dimensional Search Spaces

namespace openmore
{
class AnytimeDynamicRRT;
typedef std::shared_ptr<AnytimeDynamicRRT> AnytimeDynamicRRTPtr;

class AnytimeDynamicRRT: public DynamicRRT
{
#define FAILED_ITER 3
protected:
  bool improvePath(NodePtr &node, const double& max_time);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AnytimeDynamicRRT(Eigen::VectorXd& current_configuration,
                    PathPtr& current_path, const double& max_time,
                    const TreeSolverPtr &solver, const cnr_logger::TraceLoggerPtr &logger);

  bool replan() override;
};
}

