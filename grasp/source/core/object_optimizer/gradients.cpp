
#include <ceres/ceres.h>
#include "core/object_optimizer.hpp"
#include "utils/utils.hpp"

namespace grasp{

int ObjectPoseOptimizer::ComputeJacob() {
	
	// Gradients w.r.t. object escape constraint
	_cdist_mat_escape.array() += EPSILON;
	_dpenalty_deex = -_cdist_dx_escape.cwiseQuotient(_cdist_mat_escape);
	_dpenalty_deey = -_cdist_dy_escape.cwiseQuotient(_cdist_mat_escape);
	_dpenalty_deez = -_cdist_dz_escape.cwiseQuotient(_cdist_mat_escape);

	// resize to fit the dimension of mu
	_cdist_obj_escape_err_greater_than_zero = (_cdist_obj_escape_err.array() > 0.0).cast<double>();
	_cdist_obj_escape_err_greater_than_zero.resize(NUM_OBJ_PTS*NUM_ROBOT_MUST_TOUCH_EE, 1);
	_cdist_obj_escape_err.resize(NUM_OBJ_PTS*NUM_ROBOT_MUST_TOUCH_EE, 1);
	_dpenalty_deex.resize(NUM_OBJ_PTS*NUM_ROBOT_MUST_TOUCH_EE, 1);
	_dpenalty_deey.resize(NUM_OBJ_PTS*NUM_ROBOT_MUST_TOUCH_EE, 1);
	_dpenalty_deez.resize(NUM_OBJ_PTS*NUM_ROBOT_MUST_TOUCH_EE, 1);

	// compute derivatives of lagrangian w.r.t. end-effector (x, y, z)
	slack_var = _cdist_obj_escape_err_greater_than_zero.cwiseProduct((_mu.block(0, 0, NUM_OBJ_PTS*NUM_ROBOT_MUST_TOUCH_EE, 1) + _rho * _cdist_obj_escape_err));
	_dlgrgn_deex = slack_var.cwiseProduct(_dpenalty_deex);
	_dlgrgn_deey = slack_var.cwiseProduct(_dpenalty_deey);
	_dlgrgn_deez = slack_var.cwiseProduct(_dpenalty_deez);

	// resize them back
	_cdist_obj_escape_err.resize(NUM_OBJ_PTS, NUM_ROBOT_MUST_TOUCH_EE);
	_dlgrgn_deex.resize(NUM_OBJ_PTS, NUM_ROBOT_MUST_TOUCH_EE);
	_dlgrgn_deey.resize(NUM_OBJ_PTS, NUM_ROBOT_MUST_TOUCH_EE);
	_dlgrgn_deez.resize(NUM_OBJ_PTS, NUM_ROBOT_MUST_TOUCH_EE);

	_dlgrgn_dpts << _dlgrgn_deex.rowwise().sum(), _dlgrgn_deey.rowwise().sum(), _dlgrgn_deez.rowwise().sum();
	
	_dlgrgn_drot = _object_position_origin.transpose() * _dlgrgn_dpts; 
	_dlgrgn_dtrans = _dlgrgn_dpts.transpose().rowwise().sum();

	_dRbase_droll << 
					0, obj_cy*obj_sp*obj_cr + obj_sy*obj_sr, -obj_cy*obj_sp*obj_sr + obj_sy*obj_cr,
					0, obj_sy*obj_sp*obj_cr - obj_cy*obj_sr, -obj_sy*obj_sp*obj_sr - obj_cy*obj_cr,
					0, obj_cp*obj_cr,           -obj_cp*obj_sr;
	_dRbase_dyaw << 
					-obj_sy*obj_cp, -obj_sy*obj_sp*obj_sr - obj_cy*obj_cr, -obj_sy*obj_sp*obj_cr + obj_cy*obj_sr,
					obj_cy*obj_cp,  obj_cy*obj_sp*obj_sr - obj_sy*obj_cr,  obj_cy*obj_sp*obj_cr + obj_sy*obj_sr, 
					0,      0,                0;
	_dRbase_dpitch << 
					-obj_cy*obj_sp, obj_cy*obj_cp*obj_sr, obj_cy*obj_cp*obj_cr,
					-obj_sy*obj_sp, obj_sy*obj_cp*obj_sr, obj_sy*obj_cp*obj_cr,
					-obj_cp,    -obj_sp*obj_sr,   -obj_sp*obj_cr;

	_grad_base_euler(0) = _dlgrgn_drot.cwiseProduct(_dRbase_droll).sum();
	_grad_base_euler(1) = _dlgrgn_drot.cwiseProduct(_dRbase_dyaw).sum();
	_grad_base_euler(2) = _dlgrgn_drot.cwiseProduct(_dRbase_dpitch).sum();
	_grad_base_euler(3) = _dlgrgn_dtrans(0);
	_grad_base_euler(4) = _dlgrgn_dtrans(1);
	_grad_base_euler(5) = _dlgrgn_dtrans(2);

	// base euler norm objective
	_grad_base_euler(0) -= _trans_obj_scale * _object_rotation_euler_escape(0) / (std::sqrt(_euler_norm_square) + EPSILON);
	_grad_base_euler(1) -= _trans_obj_scale * _object_rotation_euler_escape(1) / (std::sqrt(_euler_norm_square) + EPSILON);
	_grad_base_euler(2) -= _trans_obj_scale * _object_rotation_euler_escape(2) / (std::sqrt(_euler_norm_square) + EPSILON);
	_grad_base_euler(3) -= _trans_obj_scale * _object_translation_escape(0) / (std::sqrt(_euler_norm_square) + EPSILON);
	_grad_base_euler(4) -= _trans_obj_scale * _object_translation_escape(1) / (std::sqrt(_euler_norm_square) + EPSILON);
	_grad_base_euler(5) -= _trans_obj_scale * _object_translation_escape(2) / (std::sqrt(_euler_norm_square) + EPSILON);

	// propagate to euler_var
	for (int i = 0; i < 6; ++i) 
		_grad_base_euler_var(i) = _grad_base_euler(i) * (_euler_range[i][1] - _euler_range[i][0]) 
									* _object_base_euler_norm[i] * (1 - _object_base_euler_norm[i]);
	
	return 1;
};



} // end namespace grasp
