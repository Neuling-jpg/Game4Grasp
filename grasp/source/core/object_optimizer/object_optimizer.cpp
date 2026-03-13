
#include <ceres/ceres.h>
#include "core/object_optimizer.hpp"
#include "utils/utils.hpp"

namespace grasp{


int run_player2(ObjectPoseOptimizer* solver_ptr, const std::vector<double>& theta, 
			const bool& use_cdist, const bool& _verbose) {
	
	if (_verbose) std::cout << "run_player2 starts to solve.\n";
	if (_verbose) std::cout << "Eigen will use " << Eigen::nbThreads() << " threads\n";
	
	// optimization
	ceres::GradientProblemSolver::Options _ceres_options;
    ceres::GradientProblemSolver::Summary _ceres_summary;
	_ceres_options.logging_type = ceres::SILENT;
    _ceres_options.line_search_direction_type = ceres::BFGS;

    // Initialize hyper-parameters
    int _max_loop_outer = 100;
    double _alpha = 10;
	double last_escape_penalty = INFINITY, last_euler_square = INFINITY;
    double ceres_vars[ 6 ] ;  // 6 for object base 6D euler

	/////////////////////////
	// augmented lagrangian
	/////////////////////////

	// Initialization
	solver_ptr -> InitParams();
	solver_ptr -> ObjectPointCloudPositionRollout();

	// Compute objectives, constraints, and Augmented Lagrangian
	solver_ptr -> ComputeLgrgn();

	if (_verbose) {  // print something
		std::cout << "\n\n[Initial] " << std::endl;
		std::cout << "escape norm: " << std::to_string(std::sqrt(solver_ptr -> _euler_norm_square)) << std::endl;
		std::cout << "escape constraint: " << std::to_string(solver_ptr -> _escape_penalty) << std::endl;
	}

	// Compute Jacobian
	solver_ptr -> ComputeJacob();

	for (int i_outer = 0; i_outer < _max_loop_outer; ++i_outer) {     // augmented lagrangian
		
		// initialize ceres_vars value as variables
		solver_ptr -> Var2CeresVar(ceres_vars);
        
        OptimizeObject* _ik_func = new OptimizeObject(solver_ptr);

        ceres::GradientProblem ik_opt_problem( _ik_func );
        ceres::Solve(_ceres_options, ik_opt_problem, ceres_vars, &_ceres_summary);

		if (_verbose) {  // print something
            std::cout << "\n\n[Outer Loop] " << i_outer << std::endl;
            std::cout << _ceres_summary.BriefReport() << std::endl;
			std::cout << "escape norm: " << std::to_string(std::sqrt(solver_ptr -> _euler_norm_square)) << std::endl;
            std::cout << "escape constraint: " << std::to_string(solver_ptr -> _escape_penalty) << std::endl;
        }

		if (solver_ptr -> _escape_penalty < 1e-6  // this must be SUPER strict otherwise run_player1 will fail
		) {
            if (_verbose) {
                std::cout << "[Outer loop criterion] converged after " 
                << std::to_string(i_outer+1) << " iters\n" << std::endl;
            }
            break;
        }

		if (abs(solver_ptr -> _euler_norm_square - last_euler_square) < 1e-5
			&&
			abs(solver_ptr -> _escape_penalty - last_escape_penalty) < 1e-9
		) {
            if (_verbose) {
                std::cout << "[Outer loop criterion] converged after " 
                << std::to_string(i_outer+1) << " iters\n" << std::endl;
            }
            break;
        }
		else {
			last_euler_square = solver_ptr -> _euler_norm_square;
			last_escape_penalty = solver_ptr -> _escape_penalty;
		}

		// Update Lagragians
		solver_ptr -> UpdtLgrgnWeight(_alpha);
    }

	return 1;
};

} // end namespace grasp
