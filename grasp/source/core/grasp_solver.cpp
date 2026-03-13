
#include <ceres/ceres.h>

#include "core/grasp_solver.hpp"
#include "utils/utils.hpp"

namespace grasp{

int GraspSolver(RobotConfigOptimizer* robot_solver_ptr, 
					ObjectPoseOptimizer* object_solver_ptr ,
					const std::vector<double>& theta, 
					const bool& use_cdist, const bool& _verbose) {

	int converged = 0;
	
	// initialization
	robot_solver_ptr -> InitLinkPoses();
	robot_solver_ptr -> Angle2State(theta);
	robot_solver_ptr -> _object_rotation_euler_escape << 0, 0, 0;
	robot_solver_ptr -> _object_translation_escape << 0, 0, 0;
	
	// start playing
	double last_escape_norm = INFINITY;
	for (int i = 0; i < 10; ++i) {
		if (_verbose) {
			std::cout << "================================" << std::endl;
			std::cout << "=========== run_player1 ============" << std::endl;
			std::cout << "================================" << std::endl;
		}
		
		run_player1(robot_solver_ptr, theta, use_cdist, _verbose);
		
		object_solver_ptr -> _must_touch_ee_position = robot_solver_ptr -> _active_ee_position;
		object_solver_ptr -> _must_touch_cdist_mat = robot_solver_ptr -> _colli_cdist_mat;
		object_solver_ptr -> _cdist_threshold = robot_solver_ptr -> _colli_cdist_mat_mini_tolerance;
		
		if (_verbose) {
			std::cout << "================================" << std::endl;
			std::cout << "=========== run_player2 ============" << std::endl;
			std::cout << "================================" << std::endl;
		}

		run_player2(object_solver_ptr, theta, use_cdist, _verbose);
		// run_player2(object_solver_ptr, theta, use_cdist, false);
		
		robot_solver_ptr -> _object_rotation_euler_escape = object_solver_ptr -> _object_rotation_euler_escape;
		robot_solver_ptr -> _object_translation_escape = object_solver_ptr -> _object_translation_escape;
		
		if (_verbose) {
			std::cout << "Iter " << i << ": object escape norm = " << std::sqrt(object_solver_ptr -> _euler_norm_square) << std::endl;
			std::cout << "Iter " << i << ": object escape euler = \n" << object_solver_ptr -> _object_rotation_euler_escape << "\n" 
						<< object_solver_ptr -> _object_translation_escape << std::endl;
		}
		
		if (std::sqrt(object_solver_ptr -> _euler_norm_square) < 1e-3
			&&
			robot_solver_ptr -> _form_closure_penalty < 1e-3)
			break;
		if (std::abs(std::sqrt(object_solver_ptr -> _euler_norm_square) - last_escape_norm) < 1e-5) 
			break;
		else 
			last_escape_norm = std::sqrt(object_solver_ptr -> _euler_norm_square);
	}
	
	if (converged==0) {
		if (_verbose) std::cout << "[Warning] GraspSolver did not converge, go back to pure optimize" << std::endl;
	}

	return converged;
};

int GraspSolver(RobotConfigOptimizer* robot_solver_ptr, 
				const std::vector<double>& theta, 
				const bool& use_cdist, const bool& _verbose) {
		
	// initialization
	robot_solver_ptr -> InitLinkPoses();
	robot_solver_ptr -> Angle2State(theta);
	robot_solver_ptr -> _object_rotation_euler_escape << 0, 0, 0;
	robot_solver_ptr -> _object_translation_escape << 0, 0, 0;
	run_player1(robot_solver_ptr, theta, use_cdist, _verbose);
	
	return 1;
};


} // end namespace grasp
