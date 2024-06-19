#pragma once

#include <eigen3/Eigen/Core>
#include <graph_core/solvers/tree_solver.h>
#include <graph_core/metrics/metrics_base.h>
#include <graph_core/collision_checkers/collision_checker_base.h>

namespace openmore
{
using namespace graph::core;

#ifdef ROS_AVAILABLE
#include <graph_display/graph_display.h>
using namespace graph::display;
#endif
}


