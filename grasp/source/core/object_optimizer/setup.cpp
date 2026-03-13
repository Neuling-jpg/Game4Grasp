
#include <ceres/ceres.h>
#include "core/object_optimizer.hpp"
#include "utils/utils.hpp"

namespace grasp{

ObjectPoseOptimizer::ObjectPoseOptimizer(){
	return ;
};

ObjectPoseOptimizer::~ObjectPoseOptimizer(){
	return ;
};


int ObjectPoseOptimizer::InitParams(const std::vector<int>& must_touch_obj_ee_id, 
									const std::vector<std::vector<double>>& object_pts) {

	_object_position_origin = Vec2d2Eigen(object_pts);

	NUM_OBJ_PTS = _object_position_origin.rows();
	if (NUM_OBJ_PTS==0) {
		std::cout << "[ERROR] ObjectPoseOptimizer::InitParams() NUM_OBJ_PTS==0." << std::endl;
		throw std::runtime_error("[ERROR]");
		return 0;
	}

	NUM_ROBOT_MUST_TOUCH_EE = must_touch_obj_ee_id.size();
	if (NUM_ROBOT_MUST_TOUCH_EE==0) {
		std::cout << "[ERROR] ObjectPoseOptimizer::InitParams() NUM_ROBOT_MUST_TOUCH_EE==0." << std::endl;
		throw std::runtime_error("[ERROR]");
		return 0;
	}

	InitParams();

	return 1;
};

int ObjectPoseOptimizer::InitParams() {
	
	_object_base_euler_variable = {1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3};
	_object_base_euler_norm = {0, 0, 0, 0, 0, 0};
	_object_rotation_euler_escape.resize(3, 1);
	_object_translation_escape.resize(3, 1);
	_object_rotation_mat_escape.resize(3, 3);

	_cdist_mat_escape.resize(NUM_OBJ_PTS, NUM_ROBOT_MUST_TOUCH_EE);
	_cdist_dx_escape.resize(NUM_OBJ_PTS, NUM_ROBOT_MUST_TOUCH_EE);
	_cdist_dy_escape.resize(NUM_OBJ_PTS, NUM_ROBOT_MUST_TOUCH_EE);
	_cdist_dz_escape.resize(NUM_OBJ_PTS, NUM_ROBOT_MUST_TOUCH_EE);

	_dlgrgn_dpts.resize(NUM_OBJ_PTS, 3);
	_dRbase_droll.resize(3, 3);
	_dRbase_dpitch.resize(3, 3);
	_dRbase_dyaw.resize(3, 3);
	_grad_base_euler.resize(6, 1);
	_grad_base_euler_var.resize(6, 1);

	// object transform range
	_euler_range = {
					{-0.001, 0.01},  // roll
					{-0.01, 0.01}, // pitch
					{-0.01, 0.01},  // yaw
					{-0.005, 0.005},  // x
					{-0.005, 0.005},  //y
					{-0.005, 0.005}  // z
					};
	
	_max_euler_norm_square = 0;
	for (int i = 0; i < _euler_range.size(); ++i) {
		_max_euler_norm_square += _euler_range[i][0]*_euler_range[i][0];
	}

	InitGradValue();
	InitLgrgnWeight();

	return 1;
};

int ObjectPoseOptimizer::InitGradValue() {
	_grad_base_euler_var << 0, 0, 0, 0, 0, 0;
	return 1;
};

int ObjectPoseOptimizer::InitLgrgnWeight() {
	dim_mu = NUM_ROBOT_MUST_TOUCH_EE * NUM_OBJ_PTS;
	_mu = Eigen::MatrixXd::Zero(dim_mu, 1); 
	_rho = 1;
	return 1;
};


int ObjectPoseOptimizer::UpdtLgrgnWeight(const double &alpha) {
	if (_escape_penalty > 0.0) 
		_cdist_obj_escape_err.resize(NUM_OBJ_PTS*NUM_ROBOT_MUST_TOUCH_EE, 1);
		_mu.block(0, 0, NUM_OBJ_PTS*NUM_ROBOT_MUST_TOUCH_EE, 1) += _rho * _cdist_obj_escape_err;
		_cdist_obj_escape_err.resize(NUM_OBJ_PTS, NUM_ROBOT_MUST_TOUCH_EE);
		_rho *= alpha;
	return 1;
};


int ObjectPoseOptimizer::Var2CeresVar(double* ceres_var) {
	for (int idx_ceres = 0; idx_ceres < 6; ++idx_ceres) 
		ceres_var[idx_ceres] = _object_base_euler_variable[idx_ceres];
	return 1;
};

int ObjectPoseOptimizer::CeresVar2Var(const double* ceres_var) {
	for (int idx_ceres = 0; idx_ceres < 6; ++idx_ceres) 
		_object_base_euler_variable[idx_ceres] = ceres_var[idx_ceres];
	return 1;
};

int ObjectPoseOptimizer::Grad2CeresGrad(double* ceres_grad) {
	for (int idx_ceres = 0; idx_ceres < 6; ++idx_ceres) 
		ceres_grad[idx_ceres] = _grad_base_euler_var(idx_ceres);
	return 1;
};


} // end namespace grasp
