
#include <ceres/ceres.h>
#include "core/robot_optimizer.hpp"
#include "utils/utils.hpp"

namespace grasp{

int run_player1(RobotConfigOptimizer* solver_ptr, const std::vector<double>& theta, 
			const bool& use_cdist, const bool& _verbose) {
	
	if (_verbose) std::cout << "run_player1 starts to solve.\n";
	if (_verbose) std::cout << "Eigen will use " << Eigen::nbThreads() << " threads\n";
	
	// optimization
	ceres::GradientProblemSolver::Options _ceres_options;
    ceres::GradientProblemSolver::Summary _ceres_summary;
	_ceres_options.logging_type = ceres::SILENT;
    _ceres_options.line_search_direction_type = ceres::BFGS;

    // Initialize hyper-parameters
    int _max_loop_outer = 100;
    double _alpha = 10;
	double last_colli_penalty = INFINITY, last_colli_link_penalty = INFINITY, last_max_self_colli_penalty = INFINITY,
		   last_ee_penalty = INFINITY, last_max_cdist_penalty = INFINITY,
		   last_form_closure_penalty = INFINITY;
    double ceres_vars[ 6 + solver_ptr -> num_total_sub_joints ] ;  // 6 for base frame 6D euler

	/////////////////////////
	// augmented lagrangian
	/////////////////////////

	// Initialization
	solver_ptr -> InitLgrgnWeight();
	solver_ptr -> FKRollout();
	solver_ptr -> use_cdist = use_cdist;	

	// Compute objectives, constraints, and Augmented Lagrangian
	solver_ptr -> ComputeLgrgn();
	if (_verbose) {  // print something
		std::cout << "\n\n[Initial] " << std::endl;
		std::cout << "ee constraint: " << std::to_string(solver_ptr -> _ee_penalty + solver_ptr -> _cdist_penalty) << std::endl;
		std::cout << "joint-obj collision constraint: " << std::to_string(solver_ptr -> _colli_penalty) << std::endl;
		std::cout << "link-obj collision constraint: " << std::to_string(solver_ptr -> _colli_link_penalty) << std::endl;
		std::cout << "self collision constraint: " << std::to_string(solver_ptr -> _self_colli_penalty) << std::endl;
		std::cout << "form clousre constraint: " << std::to_string(solver_ptr -> _form_closure_penalty) << std::endl;
	}

	// Compute Jacobian
	solver_ptr -> ComputeJacob();

	for (int i_outer = 0; i_outer < _max_loop_outer; ++i_outer) {     // augmented lagrangian
		
		// initialize ceres_vars value as variables
		solver_ptr -> Var2CeresVar(ceres_vars);
        
        OptimizeRobot* robot_optimize_func = new OptimizeRobot(solver_ptr);
        ceres::GradientProblem ik_opt_problem( robot_optimize_func );
        ceres::Solve(_ceres_options, ik_opt_problem, ceres_vars, &_ceres_summary);

		if (_verbose) {  // print something
            std::cout << "\n\n[Outer Loop] " << i_outer << std::endl;
            std::cout << _ceres_summary.BriefReport() << std::endl;
            std::cout << "ee constraint: " << std::to_string(solver_ptr -> _ee_penalty + solver_ptr -> _cdist_penalty) << std::endl;
            std::cout << "joint-obj collision constraint: " << std::to_string(solver_ptr -> _colli_penalty) << std::endl;
			std::cout << "link-obj collision constraint: " << std::to_string(solver_ptr -> _colli_link_penalty) << std::endl;
			std::cout << "self collision constraint: " << std::to_string(solver_ptr -> _self_colli_penalty) << std::endl;
            std::cout << "form clousre constraint: " << std::to_string(solver_ptr -> _form_closure_penalty) << std::endl;
        }

		if (solver_ptr -> _colli_penalty < 1e-3 
			&&
			solver_ptr -> _colli_link_penalty < 1e-3
			&&
			solver_ptr -> _max_self_colli_penalty < 1e-3 
			&& 
			solver_ptr -> _ee_penalty < 1e-3
			&& 
			solver_ptr -> _max_cdist_penalty < 1e-3
			&&
			solver_ptr -> _form_closure_penalty < 1e-3
		) {
            if (_verbose) {
                std::cout << "[Outer loop criterion] converged after " 
                << std::to_string(i_outer+1) << " iters\n" << std::endl;
            }
            break;
        }

		if (abs(solver_ptr -> _colli_penalty - last_colli_penalty) < 1e-6
			&&
			abs(solver_ptr -> _colli_link_penalty - last_colli_link_penalty) < 1e-6
			&&
			abs(solver_ptr -> _max_self_colli_penalty - last_max_self_colli_penalty) < 1e-6
			&& 
			abs(solver_ptr -> _ee_penalty - last_ee_penalty) < 1e-6
			&& 
			abs(solver_ptr -> _max_cdist_penalty - last_max_cdist_penalty) < 1e-6
			&&
			abs(solver_ptr -> _form_closure_penalty - last_form_closure_penalty) < 1e-6
		) {
            if (_verbose) {
                std::cout << "[Outer loop criterion] converged after " 
                << std::to_string(i_outer+1) << " iters\n" << std::endl;
            }
            break;
        }
		else {
			last_colli_penalty = solver_ptr -> _colli_penalty;
			last_colli_link_penalty = solver_ptr -> _colli_link_penalty;
			last_max_self_colli_penalty = solver_ptr -> _max_self_colli_penalty;
			last_ee_penalty = solver_ptr -> _ee_penalty;
			last_max_cdist_penalty = solver_ptr -> _max_cdist_penalty;
			last_form_closure_penalty = solver_ptr -> _form_closure_penalty;
		}

		// Update Lagragians
		solver_ptr -> UpdtLgrgnWeight(_alpha);

    }

	solver_ptr -> Var2Angle();

	return 1;
};

} // end namespace grasp
