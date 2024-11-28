## Tutorials
This tutorial provides a step-by-step guide on using a replanner to quickly adjust a robot's path when its current one becomes invalid. All necessary classes and tools for solving path planning problems are available in the [`graph_core`](https://github.com/JRL-CARI-CNR-UNIBS/graph_core) library, and [here](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/blob/master/docs/tutorial/tutorial_intro.md) you can find the related tutorials.

### Use a replanner
In [this tutorial](https://github.com/JRL-CARI-CNR-UNIBS/replanners_lib/blob/master/documentation/tutorial/tutorial1.cpp), we demonstrate the use of a path replanner from `replanners_lib` to  replan a robot's path when encountering obstacles. The tutorial involves:

1. Computing an initial path between a start and goal configuration.
2. Simulating an obstacle that invalidates the computed path.
3. Replanning the path to find a valid solution.

For detailed information about path computation using `graph_core`, visit its [tutorials page](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/blob/master/docs/tutorial/tutorial_intro.md).

1. To begin, include the relevant headers from `graph_core` and `replanners_lib`. In this example, we use the [Dynamic Rapidly-exploring Random Trees (DRRT)](https://ieeexplore.ieee.org/document/1641879) replanner.

```cpp
// graph_core required headers
#include <graph_core/solvers/rrt.h>
#include <graph_core/samplers/uniform_sampler.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/collision_checkers/cube_3d_collision_checker.h>

// replanners_lib required headers
#include <openmore/replanners/DRRT.h>
```

2. A logger is required for tracking information during execution. Use the `cnr_logger` library with a configuration file to specify logging settings. For further details, refer to the [cnr_logger repository](https://github.com/CNR-STIIMA-IRAS/cnr_logger).
```cpp
// Load the logger's cofiguration
std::string path_to_config_folder = "path/to/config/folder";
std::string logger_file = path_to_config_folder+"/logger_param.yaml";
cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("openmore_tutorial_loggers",logger_file);
```

3. Define a collision checker, a cost function, a sampler, and finally the solver.
To handle parameters, you can utilize `cnr_param`, which leverages [yaml-cpp](https://github.com/jbeder/yaml-cpp) to read parameter values from YAML files. `cnr_param` also integrates with the ROS and ROS2 parameter ecosystems, making it versatile for different applications. See the [`cnr_param`](https://github.com/CNR-STIIMA-IRAS/cnr_param) repository for further details. Additionally, `graph_core` provides a wrapper function to simplify parameter retrieval, helping to streamline the setup process. You can find the main utility functions offered by `graph_core` [here](https://github.com/JRL-CARI-CNR-UNIBS/graph_core/blob/master/graph_core/include/graph_core/util.h).

```cpp
// Define namespace for parameters retrieving
  std::string param_ns = "/openmore_tutorial";  // must begin with "/"

// Define the collision checker (foo collision checker)
double min_cc_distance;
double default_min_cc_distance = 0.01;
graph::core::get_param(logger,param_ns,"min_cc_distance",min_cc_distance,min_cc_distance); //wrapper to cnr_param functions
  
if(min_cc_distance<=0)
  min_cc_distance = default_min_cc_distance;

double joints_threshold = 0.0;

graph::collision_check::CollisionCheckerPtr collision_checker = 
std::make_shared<graph::collision_check::Cube3dCollisionChecker>(logger,joints_threshold,min_cc_distance);

// Define a cost function (Euclidean metrics)
graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);

// Define lower/upper bounds
size_t dof = 3; 
Eigen::VectorXd lb(dof); lb.setConstant(-2.5);
Eigen::VectorXd ub(dof); ub.setConstant( 2.5);

// Define a sampler (uniform sampler)
graph::core::SamplerPtr sampler = std::make_shared<graph::core::UniformSampler>(lb,ub,logger);

// Define the solver (RRT)
graph::core::TreeSolverPtr solver =
std::make_shared<graph::core::RRT>(metrics,collision_checker,sampler,logger);
```
In this example, the collision checker uses the parameter `joints_threshold` as an input. 
This parameter defines a fictitious obstacle: if the absolute value of even a single joint does not exceed `joints_threshold`, the entire configuration will be deemed to be in collision.
Currently, with the threshold set to zero, no obstacles are considered in the scene.

4. Define the start and goal nodes for the path planning process. You can specify these configurations directly, or use the `graph::core::get_param(..)` function to read the start and goal configurations from parameter files. This approach allows you to manage configurations more flexibly, especially when working with complex or varying setups.

```cpp
// Define start and goal nodes
Eigen::VectorXd start_configuration(dof);
start_configuration << -1.5,-1.5,-1.5;

Eigen::VectorXd goal_configuration(dof);
goal_configuration <<  1.5, 1.5, 1.5;

// Optionally, you can load these directly from params
// graph::core::get_param(logger,param_ns,"start_configuration",start_configuration);
// graph::core::get_param(logger,param_ns,"goal_configuration",goal_configuration)

graph::core::NodePtr start_node = std::make_shared<graph::core::Node>(start_configuration,logger);
graph::core::NodePtr goal_node  = std::make_shared<graph::core::Node>(goal_configuration, logger);
```

5. With the solver set up, compute the path from the start to the goal node.

```cpp
  // Compute the initial robot's path
  double max_time = 10.0; //seconds
  size_t max_iter = 1000000;
  graph::core::PathPtr initial_path;
  
  bool found = solver->computePath(start_node,goal_node,param_ns,initial_path,max_time,max_iter);
  
  if(found)
    CNR_INFO(logger,"Path found! Cost: "<<initial_path->cost());
  else
  {
    CNR_ERROR(logger,"Initial path not found!");
    return 0;
  }
```

6. Now, we simulate a new obstacle appearing in the scene, which invalidates the previously computed path.
 The presence of this obstacle is detected through collision checking, leading to an update in the path cost.
  If the path is blocked, the obstructed connections are assigned an infinite cost, effectively rendering the entire path invalid.

```cpp
  // Simulate a new obstacle on the path and update its cost
  joints_threshold = 1.0; //increases the obstacle's size, from 0.0 to 1.0 on each robot's joint
  graph::collision_check::CollisionCheckerPtr new_obs_collision_checker = 
    std::make_shared<graph::collision_check::Cube3dCollisionChecker>(logger,joints_threshold,min_cc_distance);
  initial_path->setCollisionChecker(new_obs_collision_checker);
  initial_path->isValid();

  CNR_INFO(logger,"Updated path's cost: "<<initial_path->cost());
```
7. To create a replanner object, the following inputs are required:
- Current robot configuration: the robot's position along the path. Replanning adjusts or replaces the path segment from the current configuration to the goal.
- Current path: the robot's current path, potentially obstructed by an obstacle. Some algorithms may replan even without obstruction to find a lower-cost solution. For instance, the DRRT algorithm will not replan if the path is unobstructed.
- Maximum replanning time: the time limit for replanning, which must be short to ensure responsiveness. For robots with many degrees of freedom in real scenarios, 200 ms is often a good balance between reactivity and the algorithm’s capacity to generate a solution.
- Replanning solver: path replanning algorithms utilize solvers from standard path planning to recover a feasible solution. For example, DRRT removes invalidated branches caused by obstacles and regrows the tree using RRT.
- Logger: a utility for logging purposes.
 ```cpp
   // Create a replanner object
  graph::core::TreeSolverPtr replanning_solver =
   std::make_shared<graph::core::RRT>(metrics,new_obs_collision_checker,sampler,logger);

  double max_replanning_time = 0.200; // 200 ms
  Eigen::VectorXd current_configuration = start_configuration;
  openmore::ReplannerBasePtr replanner =
   std::make_shared<openmore::DynamicRRT>(current_configuration,initial_path,max_time,replanning_solver,logger);
 ```
8. Finally, we run the replanning algorithm to find a new feasible solution.
 A successful outcome is indicated by a cost that is less than infinity.
 ```cpp
  // Replan
auto tic = graph_time::now();
replanner->replan();
success = replanner->getSuccess();
auto toc = graph_time::now();

double elapsed_time = graph::core::toSeconds(toc,tic);

if(success)
  CNR_INFO(logger, "New path found in "<<elapsed_time<<" seconds. Cost: "<<replanner->getReplannedPath()->cost());
else
  CNR_ERROR(logger, "No valid path found in "<<elapsed_time<<" seconds.");
```
### Use a replanner leveraging MoveIt planning scene management
See [this tutorial](https://github.com/JRL-CARI-CNR-UNIBS/openmore_ros_examples).

### Develop a new replanner
TODO