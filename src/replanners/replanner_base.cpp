#include <openmore/replanners/replanner_base.h>

namespace openmore
{
ReplannerBase::ReplannerBase(const Eigen::VectorXd& current_configuration,
                             const PathPtr& current_path,
                             const double& max_time,
                             const TreeSolverPtr &solver,
                             const cnr_logger::TraceLoggerPtr& logger):
  current_configuration_(current_configuration),
  current_path_(current_path),
  replanned_path_(current_path),
  solver_(solver),
  metrics_(solver->getMetrics()),
  checker_(solver->getChecker()),
  lb_(solver->getSampler()->getLB()),
  ub_(solver->getSampler()->getUB()),
  goal_node_(current_path->getGoalNode()),
  success_(false),
  verbose_(false),
  max_time_(max_time),
  logger_(logger)
{
#ifdef GRAPH_DISPLAY_AVAILABLE
  disp_ = nullptr;
#endif

  assert(TOLERANCE>0);
}

ReplannerBase::~ReplannerBase()
{
}

}
