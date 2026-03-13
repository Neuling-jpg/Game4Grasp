
#ifndef OBJECT_OPTIMIZER_H_
#define OBJECT_OPTIMIZER_H_

#include <vector>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utils/utils.hpp"

namespace grasp{

/**
 * @brief Object pose optimization for grasping, which is the player 2 in the game formulation.
 * The object pose optimization is to find the object pose that can escape from the grasp of the robot given the robot configuration. 
 * The object pose optimization is formulated as a constrained optimization problem, where
 */
class ObjectPoseOptimizer
{
public:
	
	/**
	 * @brief Constructor
	 */
	ObjectPoseOptimizer();
	/**
	 * @brief Destructor
	 */
	virtual ~ObjectPoseOptimizer();
	/**
	 * @brief Initialize parameters
	 */
	virtual int InitParams(const std::vector<int>& must_touch_obj_ee_id, 
							const std::vector<std::vector<double>>& object_pts);
	/**
	 * @brief Initialize parameters
	 */
	virtual int InitParams();
	/**
	 * @brief Initialize Augmented Lagrangian weights (mu and rho)
	 */
	virtual int InitLgrgnWeight();
	int dim_mu;
	int NUM_ROBOT_MUST_TOUCH_EE, NUM_OBJ_PTS;
	/**
	 * @brief Update Augmented Lagrangian weights (mu and rho)
	 */
	virtual int UpdtLgrgnWeight(const double &alpha);
	/**
	 * @brief Initialize gradient values
	 */
	virtual int InitGradValue();
    Eigen::MatrixXd _grad_base_euler, _grad_base_euler_var;
	/**
	 * @brief Compute forward rollout of the object when it is trying to escape
	 */
	virtual int ObjectPointCloudPositionRollout();
	std::vector<std::vector<double>> _euler_range;
    std::vector<double> _object_base_euler_variable, _object_base_euler_norm;
	Eigen::MatrixXd _object_position_origin,
                    _object_rotation_euler_escape, _object_rotation_mat_escape, 
					_object_translation_escape, _object_points_position_escape ;
	double obj_cr, obj_sr, obj_cy, obj_sy, obj_cp, obj_sp;
    /**
	 * @brief Compute objective of base transition norm - greater the norm, smaller the penalty
	 */
	virtual int ObjectTransformObjective();
	double _max_euler_norm_square, _euler_norm_square, _euler_norm_square_objective;
	double _trans_obj_scale = 1e-5;
	/**
	 * @brief Compute object escape constraint
	 */
	virtual int ObjectEscapeConstr() ;
	std::vector<int> _must_touch_ee_id;  // must-touch end-effector id to compute distance map
	std::vector<double> _must_touch_ee_rad;
	Eigen::MatrixXd _must_touch_ee_position;
	Eigen::MatrixXd _must_touch_cdist_mat, _cdist_mat_escape;
	Eigen::MatrixXd _must_touch_cdist_dx, _must_touch_cdist_dy, _must_touch_cdist_dz;
	Eigen::MatrixXd _cdist_dx_escape, _cdist_dy_escape, _cdist_dz_escape;
	Eigen::MatrixXd _cdist_threshold, _cdist_obj_escape_diff, _cdist_obj_escape_err;
	Eigen::MatrixXd _mtcstt;
	Eigen::MatrixXd _closest_element_idx;
	Eigen::MatrixXd _rowwise_min_cdist;
	Eigen::MatrixXd _grad_rowwsie_min_cdist;
	double _escape_penalty;
	/**
	 * @brief Compute Lagrangian
	 */
	virtual int ComputeLgrgn() ;
	double lgrgn;
	Eigen::MatrixXd _lambda, _mu;
	double _rho;
	/**
	 * @brief Compute Jacobian
	 */
	virtual int ComputeJacob() ;
	Eigen::MatrixXd _dpenalty_deex, _dpenalty_deey, _dpenalty_deez;
	Eigen::MatrixXd _dlgrgn_dpts, _dlgrgn_drot, _dlgrgn_dtrans;
	Eigen::MatrixXd _dlgrgn_deex, _dlgrgn_deey, _dlgrgn_deez;
	Eigen::MatrixXd _cdist_obj_escape_err_greater_than_zero;
	Eigen::MatrixXd slack_var; // just an intermediate variable for computation conveniece
	Eigen::MatrixXd _dRbase_droll, _dRbase_dpitch, _dRbase_dyaw;
	/**
	 * @brief Copying variables to ceres variables
	 */
	virtual int Var2CeresVar(double* ceres_var) ;
	/**
	 * @brief Copying ceres variables to variables
	 */
	virtual int CeresVar2Var(const double* ceres_var) ;
	/**
	 * @brief Copying Jacobian to ceres::gradient
	 */
	virtual int Grad2CeresGrad(double* ceres_grad) ;
};


/**
 * @brief Object pose optimization ceres function
 */
class OptimizeObject final : public ceres::FirstOrderFunction {
	public:
		OptimizeObject(ObjectPoseOptimizer* solver_ptr) {
			this -> _solver_ptr = solver_ptr;
		} ;
		// OptimizeRobot() {}
		
		~OptimizeObject() override {} ;
		
		bool Evaluate(const double* parameters,
							double* cost,
							double* gradient) const override {
			
			// update variables from ceres variables
			_solver_ptr -> CeresVar2Var(parameters);
			
			_solver_ptr -> InitGradValue();

			_solver_ptr -> ObjectPointCloudPositionRollout();

			_solver_ptr -> ComputeLgrgn();

			cost[0] = _solver_ptr -> lgrgn;

			if (gradient) {
				_solver_ptr -> ComputeJacob();
				_solver_ptr -> Grad2CeresGrad(gradient);				
			}

			return true;
		}
		int NumParameters() const override { 
			return 6;  // 6 for object base euler
		}
	
	protected:
		ObjectPoseOptimizer* _solver_ptr;
};

/**
 * @brief Run player 2 optimization, which is the object pose optimization given the robot configuration.
 */
int run_player2(ObjectPoseOptimizer* solver_ptr,
			const std::vector<double>& theta,
			const bool& use_cdist = false,
			const bool& verbose = false) ;



} // end namespace grasp


#endif  // KINEMATICS_H_