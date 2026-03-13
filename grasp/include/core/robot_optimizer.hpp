
#ifndef ROBOT_OPTIMIZER_H_
#define ROBOT_OPTIMIZER_H_

#include <vector>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utils/utils.hpp"

namespace grasp{

/**
 * @brief Forward Kinematics interface class.
 */
class ForwardKinematics
{
public:
	/**
	 * @brief Constructor
	 */
	ForwardKinematics() {};
	/**
	 * @brief Destructor
	 */
	virtual ~ForwardKinematics() {};
	/**
	 * @brief The initial pose of each link
	 */
	virtual void SetInitPoses(const std::vector< std::vector<std::vector<double> > >& m0s) = 0;
	/**
	 * @brief Set the screw axes for each joint
	 */
	virtual void SetScrewAxes(const std::vector<std::vector<double> >& slist) = 0;
	/**
	 * @brief Compute the forward kinematics
	 */
	virtual int Compute(const std::vector<double>& theta) = 0;	
	/**
	 * @brief Get the pose of a specific link
	 */
	virtual std::vector<std::vector<double> > GetLinkPose(int k=-1) = 0;
	/**
	 * @brief Initialize the DH parameters
	 */
	virtual int InitKinematicParams(std::vector<std::vector<double> >& jl,
									std::vector<double>& a,
									std::vector<double>& d,
									std::vector<double>& alpha,
									std::vector<int>& parent) = 0;
	std::vector<std::vector<double> > _base;
	std::vector<double> _base_frame_decision_var, _euler_norm, _euler;
	bool fix_base_translation = false;  // whether to fix the base translation during optimization
	
};

/**
 * @brief We compute robot kinematics using PDO formula.
 * Link: https://arxiv.org/abs/2406.11572
 */
class ForwardKinematicsPDO : public ForwardKinematics
{
public:
	/**
	 * @brief Constructor
	 */
	ForwardKinematicsPDO();
	/**
	 * @brief Destructor
	 */
	virtual ~ForwardKinematicsPDO();
	/**
	 * @brief The initial pose of each link
	 */
	virtual void SetInitPoses(const std::vector< std::vector<std::vector<double> > >& m0s) override ;
	/**
	 * @brief Set the screw axes for each joint
	 */
	virtual void SetScrewAxes(const std::vector<std::vector<double> >& slist) override {};
	/**
	 * @brief Compute the forward kinematics
	 */
	virtual int Compute(const std::vector<double>& theta) override;
	/**
	 * @brief Get the pose of a specific link
	 */
	virtual std::vector<std::vector<double> > GetLinkPose(int k=-1) override;
	/**
	 * @brief Initialize the link poses
	 */
	virtual int InitLinkPoses();
	/**
	 * @brief Initialize the kinematic parameters
	 */
	virtual int InitKinematicParams(std::vector<std::vector<double> >& jl,
									std::vector<double>& a,
									std::vector<double>& d,
									std::vector<double>& alpha,
									std::vector<int>& parent) override;
	/**
	 * @brief Roll out the forward kinematics
	 */
	virtual void FKRollout();
	/**
	 * @brief Convert angles to states
	 */
	virtual void Angle2State(const std::vector<double>& theta);

	int num_total_sub_joints;  // num of all the sub-joints together

// protected:
	double ROLL_MAX, ROLL_MIN, YAW_MAX, YAW_MIN, PITCH_MAX, PITCH_MIN;
	std::vector<double> _base_frame_decision_var, _euler_norm, _euler;

	std::vector< std::vector<std::vector<double> > > _m0s;

	std::vector< std::vector<std::vector<double> > > _link_poses;

	int _num_joints = -1;

	std::vector<int> _parent;  // parent links, key: child link id, value: parent link id
	/**
	 * @brief n: num of joints, m: num of sub-joints per joint
	 */

	std::vector<std::vector<double> > _jl;  // joint limits, n x 2
	std::vector<double> _a;  // DH parameters 'a': _a[i] indicates length of link i-1, n x 1
	std::vector<double> _d;  // DH parameters 'd': _d[i] indicates offset along previous z to the common normal, n x 1
	std::vector<double> _alpha;  // DH parameters 'alpha': _alpha[i] indicates twist angle of link i-1, n x 1
	std::vector<std::vector<std::vector<double> > > _DH; // _DH is the DH matrix, n x 4 x 4	

	int _num_sub_joints = -1;  // num of totoal sub-joints after angle decomposition
	std::vector<double> _sub_joint_counter;  // num of sub-joints for each joint, n x 1

	std::vector<std::vector<std::vector<double> > > _sub_jl;  // sub-joints limits, n x m x 2
	std::vector<std::vector<double> > _sub_a;  // sub-joints DH parameters 'a': _a[i] indicates length of link i-1, n x m
	std::vector<std::vector<double> > _sub_d;  // sub-joints DH parameters 'd': _d[i] indicates offset along previous z to the common normal, n x m
	std::vector<std::vector<double> > _sub_alpha;  // sub-joints DH parameters 'alpha': _alpha[i] indicates twist angle of link i-1, n x m
	std::vector<std::vector<double> > _sub_theta; // sub_theta, joint angle after angle decomposition, n x m
	std::vector<std::vector<std::vector<std::vector<double> > > > _sub_DH; // _sub_DH is the DH matrix of the sub joint, n x m x 4 x 4	
	std::vector<std::vector<std::vector<std::vector<double> > > > _sub_link_poses;  // _sub_link_poses is the poses of the sub joint, n x m x 4 x 4	

	std::vector<std::vector<double> > _calpha; // cos(sub_alpha), n x m
	std::vector<std::vector<double> > _salpha; // sin(sub_alpha), n x m
	std::vector<std::vector<double> > _ctheta; // cos(sub_theta), n x m
	std::vector<std::vector<double> > _stheta; // sin(sub_theta), n x m

	std::vector<std::vector<double> > _joint_decision_var; // _joint_decision_var is the decision variable, n x m
	std::vector<std::vector<double> > _dobj_dw; // _dobj_dw is the gradient of the objective w.r.t. _joint_decision_var, n x m
	
	std::vector<std::vector<double> > _SR; // _SR is the range of distance, n x m
	std::vector<std::vector<double> > _MIN_L; // _MIN_L is the minimum length of the link, n x m
	std::vector<std::vector<double> > _MAX_L; // _MAX_L is the maximum length of the link, n x m
	std::vector<std::vector<double> > _l_vlink1; // _l_vlink1 is the length of the virtual link, n x m
	std::vector<std::vector<double> > _l_vlink2; // _l_vlink2 is the length of the virtual link, n x m

	std::vector<std::vector<double> > _dist; // _dist is the distance between two reference points, n x m
	std::vector<std::vector<double> > _dist_norm; // _dist_norm is the normalized distance, n x m

	std::vector<std::vector<double> > _direction; // _direction is the direction of the distance, n x m

};

/**
 * @brief Robot configuration optimization for grasping
 * , which is the player 1 in the game formulation.
 */
class RobotConfigOptimizer : public ForwardKinematicsPDO
{
public:
	
	/**
	 * @brief Constructor
	 */
	RobotConfigOptimizer();
	/**
	 * @brief Destructor
	 */
	virtual ~RobotConfigOptimizer();
	/**
	 * @brief Initialize kinematic parameters
	 * , including joint limits, DH parameters, and parent links.
	 */
	virtual int InitKinematicParams(std::vector<std::vector<double> >& jl,
		std::vector<double>& a,
		std::vector<double>& d,
		std::vector<double>& alpha,
		std::vector<int>& parent) override;
	/**
	 * @brief Initialize Augmented Lagragian weights (mu and rho)
	 */
	virtual int InitLgrgnWeight();
	int dim_mu;
	/**
	 * @brief Update Augmented Lagragian weights (mu and rho)
	 */
	virtual int UpdtLgrgnWeight(const double &alpha);
	/**
	 * @brief Initialize gradient values
	 */
	virtual int InitGradValue();
	/**
	 * @brief Compute forward rollout and gradients
	 */
	virtual int Compute(const double* vars,
						double* cost,
						double* gradient) { return 0 ; } ;
	/**
	 * @brief We don't use Compute(const std::vector<double>& theta) for now
	 */
	virtual int Compute(const std::vector<double>& theta) override { return 0 ; } ;
	/**
	 * @brief Get end effector position
	 */
	virtual int SetEETargetPosition(const std::map<int, std::vector<double>>& ee_target);
	/**
	 * @brief Setup obstacles - 3D positions of points
	 */
	virtual int SetObstacles(const std::vector< std::vector<double> >& points,
							const std::vector<int>& obj_idx,
							const std::vector< std::vector<double> >& obj_radius);
	/**
	 * @brief Compute end-effector pose objective/cost
	 */
	virtual int EEObj3D() ;
	std::map<int, std::vector<double>> _ee_target;  // end-effector target position, key: end-effector id, value: target position
	std::vector<double> _ee_target_i;  // end-effector target position for the ith end-effector, used in the loop
	std::map<int, std::vector<double>> _ee_dxyz;  // Nee x 3, end-effector delta position error at x, y, z
	std::map<int, double> _ee_dist;  // Nee x 1, end-effector distance to target position
	double _ee_penalty = 0;
	/**
	 * @brief Setup cdist computation parameters
	 */
	virtual int SetCdistComputeParams(const std::vector<int>& solid_joint_id,
									const std::vector<double>& solid_joint_rad,
									const std::vector<std::vector<int>>& solid_link_id,
									const std::vector<double>& solid_link_rad,
									const std::vector< std::vector<double> >& object_points) ;
	/**
	 * @brief Setup cdist computation parameters
	 */
	virtual int SetCdistComputeParams(const std::vector<int>& solid_joint_id,
									const std::vector<double>& solid_joint_rad,
									const std::vector<std::vector<int>>& solid_link_id,
									const std::vector<double>& solid_link_rad,
									const std::vector< std::vector<double> >& object_points,
									const std::vector< std::vector<double> >& cdist_target) ;
	/**
	 * @brief Setup cdist computation parameters
	 */
	virtual int SetCdistComputeParams(const std::vector<int>& solid_joint_id,
									const std::vector<double>& solid_joint_rad,
									const std::vector<std::vector<int>>& solid_link_id,
									const std::vector<double>& solid_link_rad,
									const std::vector< std::vector<double> >& object_points,
									const std::vector<int>& must_touch_ee_id,
									const std::vector<double>& must_touch_ee_rad) ;
	/**
	 * @brief Setup cdist computation parameters
	 */
	virtual int SetCdistComputeParams(const std::vector<int>& solid_joint_id,
									const std::vector<double>& solid_joint_rad,
									const std::vector<std::vector<int>>& solid_link_id,
									const std::vector<double>& solid_link_rad,
									const std::vector< std::vector<double> >& object_points,
									const std::vector<int>& must_touch_ee_id,
									const std::vector<double>& must_touch_ee_rad,
									const std::vector< std::vector<double> >& cdist_target) ;
	int NUM_ACTIVE_EE, NUM_MUST_TOUCH_EE, NUM_COLLISION_LINKS, NUM_OBJ_PTS;
	/**
	 * @brief Compute end-effector cdist objective/cost
	 */
	virtual int EECdistObj() ;
	bool use_cdist = false;
	std::vector<int> _active_ee_id;  // active end-effector id to compute distance map
	Eigen::MatrixXd _active_ee_position, _object_position;
	Eigen::MatrixXd _cdist_dx, _cdist_dy, _cdist_dz;
	Eigen::MatrixXd _cdist_mat, _cdist_mat_target, _cdist_mat_err;
	double _cdist_penalty, _max_cdist_penalty;
	Eigen::MatrixXd _dpenalty_dcdist, _dcdist_deex, _dcdist_deey, _dcdist_deez;
	/**
	 * @brief Compute forward rollout of the object when it is trying to escape
	 */
	virtual int ObjectEscapeRollout();
	Eigen::MatrixXd _object_rotation_euler_escape, _object_rotation_mat_escape, 
					_object_translation_escape, _object_points_position_escape ;
	double obj_cr, obj_sr, obj_cy, obj_sy, obj_cp, obj_sp;
	/**
	 * @brief Compute end-effector cdist objective/cost
	 */
	virtual int FormClosureConstr() ;
	virtual int TouchObjective() ;
	Eigen::MatrixXd _touch_objective, _mtcgtt, 
					_must_touch_to_obj_min_dist, 
					_grad_must_touch_to_obj_min_dist;
	virtual int GetMinIdx(const Eigen::MatrixXd& A,	Eigen::MatrixXd& labelMatrix) ;
	std::vector<int> _must_touch_ee_id;  // must-touch end-effector id to compute distance map
	std::vector<double> _must_touch_ee_rad;
	Eigen::MatrixXd _must_touch_ee_position;
	Eigen::MatrixXd _must_touch_cdist_mat, _cdist_mat_escape;
	Eigen::MatrixXd _must_touch_cdist_dx, _must_touch_cdist_dy, _must_touch_cdist_dz;
	Eigen::MatrixXd _cdist_dx_escape, _cdist_dy_escape, _cdist_dz_escape;
	Eigen::MatrixXd _cdist_threshold, _cdist_obj_escape_diff, _cdist_obj_escape_err;
	Eigen::MatrixXd _cdist_obj_escape_err_greater_than_zero;
	Eigen::MatrixXd _mtcstt, _coesdgtz;
	Eigen::MatrixXd _closest_element_idx;
	Eigen::MatrixXd _rowwise_min_cdist, _is_min_label;
	Eigen::MatrixXd _grad_rowwsie_min_cdist;
	double _form_closure_penalty;
	/**
	 * @brief Compute self collision constraints
	 */
	virtual int SelfCollisonConstr() ;
	std::vector<double> _active_ee_rad;
	Eigen::MatrixXd _self_cdist_dx, _self_cdist_dy, _self_cdist_dz;
	Eigen::MatrixXd _self_cdist_mat, _self_cdist_mat_mini_tolerance, 
					_self_cdist_mat_diff, _self_cdist_mat_err;
	double _self_colli_penalty, _max_self_colli_penalty;
	/**
	 * @brief Compute obstacle collision constraints
	 */
	virtual int JointObjectCollisionConstr() ;
	Eigen::MatrixXd _colli_cdist_dx, _colli_cdist_dy, _colli_cdist_dz;
	Eigen::MatrixXd _colli_cdist_mat, _colli_cdist_mat_mini_tolerance, 
					_colli_cdist_mat_diff, _colli_cdist_mat_err;
	double _colli_penalty, _max_colli_penalty;
	std::vector< std::vector<double> > _obstacle_pts;
	std::vector<int> _colli_object_idx;
	std::vector< std::vector<double> > _colli_object_radius;
	std::vector< std::vector<double> > _ellips_colli_penalty, _sphere_colli_penalty;
	double _ellips_colli_penalty_single, _sphere_colli_penalty_single;
	std::vector<double> _pt;
	int _iobj;
	std::vector< std::vector<double> > _dx_obstacle_object1, _dy_obstacle_object1, _dz_obstacle_object1, 
										_dx_obstacle_object2, _dy_obstacle_object2, _dz_obstacle_object2;
	std::vector< std::vector<double> > _dist_obstacle_object_square1, _dist_obstacle_object_square2, 
										_dist_obstacle_object1, _dist_obstacle_object2;
	/**
	 * @brief Compute obstacle collision constraints
	 */
	virtual int LinkObjectCollisionConstr() ;
	std::vector<std::vector<int>> _colli_link_id;  // link id for collision, 2 x Nlink
	std::vector<Eigen::MatrixXd> _colli_link_position;  // link position for collision, 2 x Nlink x 3
	Eigen::MatrixXd _colli_link_cdist_dx0, _colli_link_cdist_dy0, _colli_link_cdist_dz0;
	Eigen::MatrixXd _colli_link_cdist_dx1, _colli_link_cdist_dy1, _colli_link_cdist_dz1;
	Eigen::MatrixXd _colli_link_cdist_mat0, _colli_link_cdist_mat1,
					_colli_link_cdist_mat_mini_tolerance, _colli_link_cdist_mat_err;
	double _colli_link_penalty, _max_link_colli_penalty;
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
	Eigen::MatrixXd _dpenalty_deex0, _dpenalty_deey0, _dpenalty_deez0,
					_dpenalty_deex1, _dpenalty_deey1, _dpenalty_deez1;
	Eigen::MatrixXd _dpenalty_deex_rmin_filtered, 
					_dpenalty_deey_rmin_filtered, 
					_dpenalty_deez_rmin_filtered;
	Eigen::MatrixXd _dlgrgn_deex, _dlgrgn_deey, _dlgrgn_deez;
	Eigen::MatrixXd _dlgrgn_deex0, _dlgrgn_deey0, _dlgrgn_deez0,
					_dlgrgn_deex1, _dlgrgn_deey1, _dlgrgn_deez1;
	Eigen::MatrixXd _self_cdist_mat_err_greater_than_zero,
					_colli_cdist_mat_err_greater_than_zero,
					_colli_link_cdist_mat_err_greater_than_zero;
	Eigen::MatrixXd slack_var; // just an intermediate variable for computation conveniece
	/**
	 * @brief Gradient propagation
	 */
	virtual int GradPropagate(const int& i, const int& j, const int& i_prv, const int& j_prv) ;
	std::vector<std::vector<std::vector<std::vector<double> > > > _dLgrgn_dpose; // _dLgrgn_dpose is the gradient of the Lagrangian w.r.t. link pose, n x m x 4 x 4	
	std::vector<std::vector<std::vector<std::vector<double> > > > _dLgrgn_dDH; // _dobj_dDH is the gradient of the Lagrangian w.r.t. DH, n x m x 4 x 4
	std::vector<std::vector<std::vector<std::vector<double> > > > _dDH_dctheta; // _dDH_dctheta is the gradient of DH w.r.t. cos(theta), n x m x 4 x 4
	std::vector<std::vector<double> > _dctheta_ddist, _ddist_dw, _dstheta_dctheta, _ddistnorm_dw; // gradients, n x m
	std::vector<std::vector<double> > _dLgrgn_dctheta, _dLgrgn_ddist; // gradients, n x m
	double _aug_lgrgn_ratio;
	/**
	 * @brief Gradient of base frame
	 */
	virtual int BaseGradCompute() ;
	std::vector<std::vector<double>> _dLgrgn_dbase; // _dLgrgn_dbase is the gradient of the Lagrangian w.r.t. base frame pose
	std::vector<double> _grad_base_euler, _grad_base_euler_angle;
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
	/**
	 * @brief Convert variables to joint angles
	 */
	virtual int Var2Angle() ;
	std::vector<double> joint_solution;		
	/**
	 * @brief Get base frame as quaternion
	 */
	std::vector<double> GetBaseQuaternion() ;
	/**
	 * @brief Compute object escape objective/cost
	 */
	virtual int ObjEscapeConstr() ;
	double _escape_penalty;
};

/**
 * @brief IK Ceres Function
 */
class OptimizeRobot final : public ceres::FirstOrderFunction {
	public:
		OptimizeRobot(RobotConfigOptimizer* solver_ptr) {
			this -> _solver_ptr = solver_ptr;
		} ;
		
		~OptimizeRobot() override {} ;
		
		bool Evaluate(const double* parameters,
							double* cost,
							double* gradient) const override {
			
			// update variables from ceres variables
			_solver_ptr -> CeresVar2Var(parameters);
			_solver_ptr -> InitGradValue();
			_solver_ptr -> FKRollout();
			_solver_ptr -> ComputeLgrgn();
			cost[0] = _solver_ptr -> lgrgn;

			if (gradient) {
				_solver_ptr -> ComputeJacob();
				_solver_ptr -> Grad2CeresGrad(gradient);
			}
			return true;
		}
		int NumParameters() const override { 
			return 6 + _solver_ptr->num_total_sub_joints;  // 6 for base frame euler
		}
	
	protected:
		RobotConfigOptimizer* _solver_ptr;
};

/**
 * @brief Run player 1 optimization
 * , which is the robot configuration optimization given the object pose.
 */
int run_player1(RobotConfigOptimizer* solver_ptr,
			const std::vector<double>& theta,
			const bool& use_cdist = false,
			const bool& verbose = false) ;
			
} // end namespace grasp


#endif  // ROBOT_OPTIMIZER_H_
