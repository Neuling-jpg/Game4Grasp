
#include <ceres/ceres.h>
#include "core/robot_optimizer.hpp"
#include "utils/utils.hpp"

namespace grasp{

RobotConfigOptimizer::RobotConfigOptimizer(){
	return ;
};

RobotConfigOptimizer::~RobotConfigOptimizer(){
	return ;
};

int RobotConfigOptimizer::InitKinematicParams(std::vector<std::vector<double> >& jl,
	std::vector<double>& a, std::vector<double>& d, std::vector<double>& alpha, std::vector<int>& parent) {
	// Initialize the kinematic parameters for the PDO IK
	// jl: joint limits, n x 2
	// a: DH parameters 'a', n x 1
	// d: DH parameters 'd', n x 1
	// alpha: DH parameters 'alpha', n x 1
	// parent: parent link index, e.g., parent[1] = 0, parent[2] = 1, etc.
	// Note: this function is called after the robot model is initialized, so the _num_joints is set
	//       and the joint limits, DH parameters, and parent links are set.
	//       This function is used to initialize the kinematic parameters for the IK solver.
	if (_num_joints < 0){
		std::cout << "[ERROR] IK_PDO::InitKinematicParams joints and axes are not set yet!" << std::endl;
		throw std::runtime_error("[ERROR]");
		return 0;
	}

	ForwardKinematicsPDO::InitKinematicParams(jl, a, d, alpha, parent);
	_dLgrgn_dbase = MatEye(4);
	_grad_base_euler.resize(6);
	_grad_base_euler_angle.resize(3);
	_dLgrgn_dpose.resize(_num_joints);
	_dLgrgn_dDH.resize(_num_joints);
	_dDH_dctheta.resize(_num_joints);
	_dctheta_ddist.resize(_num_joints);
	_ddist_dw.resize(_num_joints);
	_dstheta_dctheta.resize(_num_joints);
	_ddistnorm_dw.resize(_num_joints);
	_dLgrgn_dctheta.resize(_num_joints);
	_dLgrgn_ddist.resize(_num_joints);
	_dDH_dctheta.resize(_num_joints);

	for (int i = 0; i < _num_joints; ++i){

		_dLgrgn_dpose[i].resize(_sub_joint_counter[i]);
		_dLgrgn_dDH[i].resize(_sub_joint_counter[i]);
		_dDH_dctheta[i].resize(_sub_joint_counter[i]);
		_dctheta_ddist[i].resize(_sub_joint_counter[i]);
		_ddist_dw[i].resize(_sub_joint_counter[i]);
		_dstheta_dctheta[i].resize(_sub_joint_counter[i]);
		_ddistnorm_dw[i].resize(_sub_joint_counter[i]);
		_dLgrgn_dctheta[i].resize(_sub_joint_counter[i]);
		_dLgrgn_ddist[i].resize(_sub_joint_counter[i]);
		_dDH_dctheta[i].resize(_sub_joint_counter[i]);

	}

	InitGradValue();

	return 1;
};

int RobotConfigOptimizer::InitGradValue() {

	for (int i = 0; i < 6; ++i) _grad_base_euler[i] = 0;

	for (int i = 0; i < _num_joints; ++i){
		
		for (int j = 0; j < _sub_joint_counter[i]; ++j){

			_dLgrgn_dpose[i][j] = MatInit(4,4,0);
			_dLgrgn_dDH[i][j] = MatInit(4,4,0);
			_dDH_dctheta[i][j] = MatInit(4,4,0);
			_dobj_dw[i][j] = 0;

		}
	
	}

	return 1;

};

int RobotConfigOptimizer::SetEETargetPosition(const std::map<int, std::vector<double>>& ee_target) {
	// initialize the end effector target position
	_ee_target = ee_target;
	// initialize the end effector delta position at x, y, z
	for (auto & pair : _ee_target) {
		_ee_dxyz[pair.first].resize(3, 0);
		_ee_dist[pair.first] = 0;
	} 
	return 1;
};

int RobotConfigOptimizer::SetCdistComputeParams(const std::vector<int>& solid_joint_id,
												const std::vector<double>& solid_joint_rad,
												const std::vector<std::vector<int>>& solid_link_id,
												const std::vector<double>& solid_link_rad,
												const std::vector< std::vector<double> >& object_points) {
	NUM_ACTIVE_EE = solid_joint_id.size();
	NUM_OBJ_PTS = object_points.size();
	NUM_COLLISION_LINKS = solid_link_id.size();

	_active_ee_id = solid_joint_id;
	
	// for end effector cdist computation
	_active_ee_position.resize(NUM_ACTIVE_EE, 3);
	_object_position = Vec2d2Eigen(object_points);
	_cdist_mat.resize(NUM_ACTIVE_EE, NUM_OBJ_PTS);
	_cdist_mat_err.resize(NUM_ACTIVE_EE, NUM_OBJ_PTS);
	
	// for self-collision computation
	_active_ee_rad = solid_joint_rad;
	_self_cdist_mat.resize(NUM_ACTIVE_EE, NUM_ACTIVE_EE);
	_self_cdist_mat_mini_tolerance.resize(NUM_ACTIVE_EE, NUM_ACTIVE_EE);
	_self_cdist_mat_err.resize(NUM_ACTIVE_EE, NUM_ACTIVE_EE);
	_self_cdist_dx.resize(NUM_ACTIVE_EE, NUM_ACTIVE_EE);
	_self_cdist_dy.resize(NUM_ACTIVE_EE, NUM_ACTIVE_EE);
	_self_cdist_dz.resize(NUM_ACTIVE_EE, NUM_ACTIVE_EE);
	// update _self_cdist_mat_mini_tolerance
	int i = 0, j = 0;
	for (auto & idee : _active_ee_id) {
		j = 0;
		for (auto & jdee : _active_ee_id) {
			if (i==j) _self_cdist_mat_mini_tolerance(i, j) = -EPSILON;
			else _self_cdist_mat_mini_tolerance(i, j) = solid_joint_rad[i] + solid_joint_rad[j];
			j++;
		}
		i++;
	}

	// for joint-object collision computation
	_colli_cdist_mat.resize(NUM_ACTIVE_EE, NUM_OBJ_PTS);
	_colli_cdist_mat_mini_tolerance.resize(NUM_ACTIVE_EE, NUM_OBJ_PTS);
	_colli_cdist_mat_err.resize(NUM_ACTIVE_EE, NUM_OBJ_PTS);
	_colli_cdist_dx.resize(NUM_ACTIVE_EE, NUM_OBJ_PTS);
	_colli_cdist_dy.resize(NUM_ACTIVE_EE, NUM_OBJ_PTS);
	_colli_cdist_dz.resize(NUM_ACTIVE_EE, NUM_OBJ_PTS);
	// update _colli_cdist_mat_mini_tolerance
	for (int i = 0; i < _active_ee_id.size(); ++i) {
		for (int j = 0; j < NUM_OBJ_PTS; ++j) {
			_colli_cdist_mat_mini_tolerance(i, j) = solid_joint_rad[i];
		}
	}

	// for link-object collision computation
	_colli_link_id = solid_link_id;
	_colli_link_position.resize(2);
	_colli_link_position[0].resize(NUM_COLLISION_LINKS, 3);
	_colli_link_position[1].resize(NUM_COLLISION_LINKS, 3);
	_colli_link_cdist_mat0.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_colli_link_cdist_mat1.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_colli_link_cdist_mat_mini_tolerance.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_colli_link_cdist_mat_err.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_colli_link_cdist_dx0.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_colli_link_cdist_dy0.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_colli_link_cdist_dz0.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_colli_link_cdist_dx1.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_colli_link_cdist_dy1.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_colli_link_cdist_dz1.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	// update _colli_link_cdist_mat_mini_tolerance
	for (int i = 0; i < solid_link_id.size(); ++i) {
		double ellip_c = 0.5 * std::sqrt(_a[solid_link_id[i][1]] * _a[solid_link_id[i][1]] 
								   		+_d[solid_link_id[i][1]] * _d[solid_link_id[i][1]]);
		double a_square = solid_link_rad[i] * solid_link_rad[i] + ellip_c * ellip_c;
		for (int j = 0; j < NUM_OBJ_PTS; ++j) {
			_colli_link_cdist_mat_mini_tolerance(i, j) = 2 * std::sqrt(a_square);
		}
	}
	
	return 1;
};

int RobotConfigOptimizer::SetCdistComputeParams(const std::vector<int>& solid_joint_id,
												const std::vector<double>& solid_joint_rad,
												const std::vector<std::vector<int>>& solid_link_id,
												const std::vector<double>& solid_link_rad,
												const std::vector< std::vector<double> >& object_points,
												const std::vector< std::vector<double> >& cdist_target) {
	
	SetCdistComputeParams(solid_joint_id, solid_joint_rad, solid_link_id, solid_link_rad, object_points) ;
	_cdist_mat_target = Vec2d2Eigen(cdist_target);

	return 1;
};

int RobotConfigOptimizer::SetCdistComputeParams(const std::vector<int>& solid_joint_id,
												const std::vector<double>& solid_joint_rad,
												const std::vector<std::vector<int>>& solid_link_id,
												const std::vector<double>& solid_link_rad,
												const std::vector< std::vector<double> >& object_points,
												const std::vector<int>& must_touch_ee_id,
												const std::vector<double>& must_touch_ee_rad) {
	
	SetCdistComputeParams(solid_joint_id, solid_joint_rad, solid_link_id, solid_link_rad, object_points) ;
	
	NUM_MUST_TOUCH_EE = must_touch_ee_id.size();
	
	_must_touch_ee_id = must_touch_ee_id;
	_must_touch_ee_rad = must_touch_ee_rad;
	_must_touch_ee_position.resize(NUM_MUST_TOUCH_EE, 3);
	_must_touch_cdist_mat.resize(NUM_MUST_TOUCH_EE, NUM_OBJ_PTS);
	_must_touch_cdist_dx.resize(NUM_MUST_TOUCH_EE, NUM_MUST_TOUCH_EE); 
	_must_touch_cdist_dy.resize(NUM_MUST_TOUCH_EE, NUM_MUST_TOUCH_EE); 
	_must_touch_cdist_dz.resize(NUM_MUST_TOUCH_EE, NUM_MUST_TOUCH_EE);
	_object_rotation_euler_escape.resize(3, 1);
	_object_rotation_mat_escape.resize(3, 3);
	_object_translation_escape.resize(3, 1);
	_object_points_position_escape.resize(NUM_OBJ_PTS, 3);
	_cdist_threshold.resize(NUM_MUST_TOUCH_EE, NUM_OBJ_PTS);
	_cdist_obj_escape_diff.resize(NUM_MUST_TOUCH_EE, NUM_OBJ_PTS);
	_rowwise_min_cdist.resize(NUM_MUST_TOUCH_EE, 1);
	_grad_rowwsie_min_cdist.resize(NUM_MUST_TOUCH_EE, NUM_OBJ_PTS);

	Eigen::MatrixXd _cdist_threshold_vec;
	_cdist_threshold_vec.resize(NUM_MUST_TOUCH_EE, 1);
	for (int i = 0; i < NUM_MUST_TOUCH_EE; i++) _cdist_threshold_vec(i, 0) = _must_touch_ee_rad[i];
	_cdist_threshold = _cdist_threshold_vec.replicate(1, NUM_OBJ_PTS);

	return 1;
};

int RobotConfigOptimizer::SetCdistComputeParams(const std::vector<int>& solid_joint_id,
												const std::vector<double>& solid_joint_rad,
												const std::vector<std::vector<int>>& solid_link_id,
												const std::vector<double>& solid_link_rad,
												const std::vector< std::vector<double> >& object_points,
												const std::vector<int>& must_touch_ee_id,
												const std::vector<double>& must_touch_ee_rad,
												const std::vector< std::vector<double> >& cdist_target) {
	
	SetCdistComputeParams(solid_joint_id, solid_joint_rad, solid_link_id, solid_link_rad, 
							object_points, must_touch_ee_id, must_touch_ee_rad) ;
	_cdist_mat_target = Vec2d2Eigen(cdist_target);

	return 1;
};

int RobotConfigOptimizer::SetObstacles(
	const std::vector< std::vector<double> >& points,
	const std::vector<int>& obj_idx,
	const std::vector< std::vector<double> >& obj_radius) {
	_obstacle_pts = points;
	_colli_object_idx = obj_idx;
	_colli_object_radius = obj_radius;

	if (_colli_object_idx.size() > 0 && _obstacle_pts.size() > 0) {
		_ellips_colli_penalty.resize(_colli_object_idx.size());

		for (int i = 0; i < _colli_object_idx.size(); ++i) {
			_ellips_colli_penalty[i].resize(_obstacle_pts.size(), 0);
		}
	}

	// initialize by value copy
	_sphere_colli_penalty = _ellips_colli_penalty;
	_dx_obstacle_object1 = _ellips_colli_penalty;
	_dy_obstacle_object1 = _ellips_colli_penalty;
	_dz_obstacle_object1 = _ellips_colli_penalty;
	_dx_obstacle_object2 = _ellips_colli_penalty;
	_dy_obstacle_object2 = _ellips_colli_penalty;
	_dz_obstacle_object2 = _ellips_colli_penalty;
	_dist_obstacle_object_square1 = _ellips_colli_penalty;
	_dist_obstacle_object_square2 = _ellips_colli_penalty;
	_dist_obstacle_object1 = _ellips_colli_penalty;
	_dist_obstacle_object2 = _ellips_colli_penalty;
	
	return 1;
}

int RobotConfigOptimizer::InitLgrgnWeight() {
	dim_mu = NUM_ACTIVE_EE * NUM_ACTIVE_EE 
			+ NUM_ACTIVE_EE * NUM_OBJ_PTS
			+ NUM_COLLISION_LINKS * NUM_OBJ_PTS 
			+ NUM_MUST_TOUCH_EE;
	_mu = Eigen::MatrixXd::Zero(dim_mu, 1); 
	_rho = 1;
	return 1;
};


int RobotConfigOptimizer::UpdtLgrgnWeight(const double &alpha) {
	
	int start_id = 0;
	
	// update mu w.r.t. self collision part
	_self_cdist_mat_err.resize(NUM_ACTIVE_EE * NUM_ACTIVE_EE, 1);
	_mu.block(start_id, 0, NUM_ACTIVE_EE * NUM_ACTIVE_EE, 1) += _rho * _self_cdist_mat_err;
	_self_cdist_mat_err.resize(NUM_ACTIVE_EE, NUM_ACTIVE_EE);
	start_id += NUM_ACTIVE_EE * NUM_ACTIVE_EE;

	// update mu w.r.t. joint-object collision part
	_colli_cdist_mat_err.resize(NUM_ACTIVE_EE * NUM_OBJ_PTS, 1);
	_mu.block(start_id, 0, NUM_ACTIVE_EE * NUM_OBJ_PTS, 1) += _rho * _colli_cdist_mat_err;
	_colli_cdist_mat_err.resize(NUM_ACTIVE_EE, NUM_OBJ_PTS);
	start_id += NUM_ACTIVE_EE * NUM_OBJ_PTS;

	// update mu w.r.t. link-object collision part
	_colli_link_cdist_mat_err.resize(NUM_COLLISION_LINKS * NUM_OBJ_PTS, 1);
	_mu.block(start_id, 0, NUM_COLLISION_LINKS * NUM_OBJ_PTS, 1) += _rho * _colli_link_cdist_mat_err;	
	_colli_link_cdist_mat_err.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	start_id += NUM_COLLISION_LINKS * NUM_OBJ_PTS;

	// update mu w.r.t. form closure part
	if (_form_closure_penalty > 0.0) {
		_mu.block(start_id, 0, NUM_MUST_TOUCH_EE, 1) += _rho * _rowwise_min_cdist;
	}
	start_id += NUM_MUST_TOUCH_EE;

	// update rho
	// Note: rho is a hyper-parameter that controls the weight of the penalty term in
	// the Lagrangian relaxation method. It is updated by multiplying with alpha.
	// alpha is a hyper-parameter that controls the step size of the optimization.
	_rho *= alpha;

	return 1;
};


int RobotConfigOptimizer::Var2CeresVar(double* ceres_var) {
	for (int idx_ceres = 0; idx_ceres < 6; ++idx_ceres) {
		ceres_var[idx_ceres] = _base_frame_decision_var[idx_ceres];
	}
	int idx_ceres = 6;
	for (int id = 0; id < _num_joints; ++id) {
		for (int jd = 0; jd < _sub_joint_counter[id]; ++jd) {
			ceres_var[idx_ceres++] = _joint_decision_var[id][jd];
		}
	}

	return 1;
};

int RobotConfigOptimizer::CeresVar2Var(const double* ceres_var) {
	for (int idx_ceres = 0; idx_ceres < 6; ++idx_ceres) {
		_base_frame_decision_var[idx_ceres] = ceres_var[idx_ceres];
	}
	int idx_ceres = 6;
	for (int id = 0; id < _num_joints; ++id) {
		for (int jd = 0; jd < _sub_joint_counter[id]; ++jd) {
			_joint_decision_var[id][jd] = ceres_var[idx_ceres++] ;
		}
	}

	return 1;
};

int RobotConfigOptimizer::Grad2CeresGrad(double* ceres_grad) {
	for (int idx_ceres = 0; idx_ceres < 6; ++idx_ceres) {
		ceres_grad[idx_ceres] = _grad_base_euler[idx_ceres];
	}
	int idx_ceres = 6;
	for (int id = 0; id < _num_joints; ++id) {
		for (int jd = 0; jd < _sub_joint_counter[id]; ++jd) {
			ceres_grad[idx_ceres++] = _dobj_dw[id][jd] ; 
		}
	}

	return 1;
};

int RobotConfigOptimizer::Var2Angle() {
	joint_solution = {};
    for (int id = 0; id < _num_joints; ++id) {
        double angle = 0, max_angle = 0, min_angle = 0;
        for (int jd = 0; jd < _sub_joint_counter[id]; ++jd) {
            if (_sub_jl[id][jd][1] == _sub_jl[id][jd][0]) { // max_subangle == min_subangle
                angle += _sub_jl[id][jd][0];
            }
            else {
                angle += std::acos(_ctheta[id][jd]);
            }
            max_angle += _sub_jl[id][jd][1];
            min_angle += _sub_jl[id][jd][0];
        }
        joint_solution.emplace_back(angle);
    }
    return 1;
};

std::vector<double> RobotConfigOptimizer::GetBaseQuaternion() {
	return transformMatrixToQuaternion(_base);
};



} // end namespace grasp
