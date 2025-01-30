/*
Copyright (c) 2024

JRL-CARI CNR-STIIMA/UNIBS
Cesare Tonola, c.tonola001@unibs.it
Manuel Beschi, manuel.beschi@unibs.it

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <openmore/replanners/replanner_base.h>

namespace openmore
{
ReplannerBase::ReplannerBase(const Eigen::VectorXd& current_configuration, const PathPtr& current_path,
                             const double& max_time, const TreeSolverPtr& solver, const TraceLoggerPtr& logger)
  : current_configuration_(current_configuration)
  , current_path_(current_path)
  , replanned_path_(current_path)
  , solver_(solver)
  , metrics_(solver->getMetrics())
  , checker_(solver->getChecker())
  , lb_(solver->getSampler()->getLB())
  , ub_(solver->getSampler()->getUB())
  , goal_node_(current_path->getGoalNode())
  , success_(false)
  , verbose_(false)
  , max_time_(max_time)
  , logger_(logger)
{
#ifdef GRAPH_DISPLAY_AVAILABLE
  disp_ = nullptr;
#endif

  assert(TOLERANCE > 0);
}

ReplannerBase::~ReplannerBase()
{
}

}  // namespace openmore
