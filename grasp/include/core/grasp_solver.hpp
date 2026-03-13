
#ifndef GRASP_SOLVER_H_
#define GRASP_SOLVER_H_

#include <vector>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utils/utils.hpp"
#include "core/robot_optimizer.hpp"
#include "core/object_optimizer.hpp"

namespace grasp{

/**
 * @brief Compute grasp by modeling it as a two-player game and solving it using IBR(Iterative Best Reaction)
 * Player 1: robot configuration optimization, given object pose
 * Player 2: object pose optimization, given robot configuration
 * The two players iteratively optimize their own objective until convergence.
 * Note: we can also directly optimize the robot configuration without modeling it as a game, which
 */
int GraspSolver(RobotConfigOptimizer* solver_ptr,
				ObjectPoseOptimizer* object_solver_ptr ,
				const std::vector<double>& theta,
				const bool& use_cdist = false,
				const bool& verbose = false) ;

/**
 * @brief Compute grasp by directly optimizing the robot configuration without modeling it as a game
 * Note: this is a simpler version of GraspSolver, 
 * which only optimizes the robot configuration without considering the object pose optimization.
 */
int GraspSolver(RobotConfigOptimizer* robot_solver_ptr, 
	const std::vector<double>& theta, 
	const bool& use_cdist, const bool& _verbose) ;

} // end namespace grasp


#endif  // GRASP_SOLVER_H_