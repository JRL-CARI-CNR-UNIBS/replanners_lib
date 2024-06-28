#pragma once

#include <eigen3/Eigen/Core>
#include <graph_core/solvers/tree_solver.h>
#include <graph_core/metrics/metrics_base.h>
#include <graph_core/collision_checkers/collision_checker_base.h>

#ifdef GRAPH_DISPLAY_AVAILABLE
#include <graph_display/graph_display.h>
#endif

namespace openmore
{
using namespace graph::core;
using namespace cnr_logger;

#ifdef GRAPH_DISPLAY_AVAILABLE
using namespace graph::display;
#endif
}


