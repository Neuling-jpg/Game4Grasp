
#include <ceres/ceres.h>
#include "core/object_optimizer.hpp"
#include "utils/utils.hpp"

namespace grasp{

int ObjectPoseOptimizer::ObjectPointCloudPositionRollout() {

    // euler based on the variables
	for (int i = 0; i < 6; ++i) _object_base_euler_norm[i] = Sigmoid(_object_base_euler_variable[i]);
    _object_rotation_euler_escape(0) = (_euler_range[0][1] - _euler_range[0][0])*_object_base_euler_norm[0] + _euler_range[0][0];  // +-0.1 rad
    _object_rotation_euler_escape(1) = (_euler_range[1][1] - _euler_range[1][0])*_object_base_euler_norm[1] + _euler_range[1][0];  // +-0.1 rad
    _object_rotation_euler_escape(2) = (_euler_range[2][1] - _euler_range[2][0])*_object_base_euler_norm[2] + _euler_range[2][0];  // +-0.1 rad
    _object_translation_escape(0) = (_euler_range[3][1] - _euler_range[3][0])*_object_base_euler_norm[3] + _euler_range[3][0];  // +- 0.005m
    _object_translation_escape(1) = (_euler_range[4][1] - _euler_range[4][0])*_object_base_euler_norm[4] + _euler_range[4][0];  // +- 0.005m
    _object_translation_escape(2) = (_euler_range[5][1] - _euler_range[5][0])*_object_base_euler_norm[5] + _euler_range[5][0];  // +- 0.005m

    // Precompute trigonometric values
    obj_cr = cos(_object_rotation_euler_escape(0)); obj_sr = sin(_object_rotation_euler_escape(0));
	obj_cy = cos(_object_rotation_euler_escape(1)); obj_sy = sin(_object_rotation_euler_escape(1));
    obj_cp = cos(_object_rotation_euler_escape(2)); obj_sp = sin(_object_rotation_euler_escape(2));
    
	_object_rotation_mat_escape << obj_cy * obj_cp, obj_cy * obj_sp * obj_sr - obj_sy * obj_cr, obj_cy * obj_sp * obj_cr + obj_sy * obj_sr,
								   obj_sy * obj_cp, obj_sy * obj_sp * obj_sr + obj_cy * obj_cr, obj_sy * obj_sp * obj_cr - obj_cy * obj_sr,
								   -obj_sp        , obj_cp * obj_sr                           , obj_cp * obj_cr                           ;
	
	// forward rollout
	_object_points_position_escape = _object_position_origin * _object_rotation_mat_escape 
										+ _object_translation_escape.transpose().replicate(NUM_OBJ_PTS, 1);

    return 1;
}


int ObjectPoseOptimizer::ObjectTransformObjective() {
    _euler_norm_square = _object_rotation_euler_escape.cwiseProduct(_object_rotation_euler_escape).sum()
				+ _object_translation_escape.cwiseProduct(_object_translation_escape).sum();
	_euler_norm_square_objective = std::sqrt(_max_euler_norm_square) - std::sqrt(_euler_norm_square);
	_euler_norm_square_objective *= _trans_obj_scale;
    return 1;
}


int ObjectPoseOptimizer::ObjectEscapeConstr() {

	// Compute _cdist_mat based on _object_transition_escape
	DistMat(_object_points_position_escape, _must_touch_ee_position, _cdist_mat_escape, 
			_cdist_dx_escape, _cdist_dy_escape, _cdist_dz_escape);
	
	// Check if:
	// for all _cdist_mat_escape > min(_cdist_mat, threshold)
	// then the object is having a valid escape
	_cdist_obj_escape_diff = _must_touch_cdist_mat.cwiseMin(_cdist_threshold).transpose() - _cdist_mat_escape;

	// compute penalty, we check the smallest element in each row in _cdist_obj_escape_diff,
	// which means the smallest difference of each end-effector w.r.t. all of the object points
	// if all smallest differences are greater than 0, then the object is okay to escape,
	// and the grasp is not stable, we therefore penalize the robot
	Relu(_cdist_obj_escape_diff, _cdist_obj_escape_err);
	_escape_penalty = _cdist_obj_escape_err.sum();		

	return 1;
};


int ObjectPoseOptimizer::ComputeLgrgn() {
	
	lgrgn = 0;
    
	// object transform objective
    ObjectTransformObjective();
	lgrgn += _euler_norm_square_objective;

	// object escape constraint
	ObjectEscapeConstr();

	_cdist_obj_escape_err.resize(NUM_OBJ_PTS*NUM_ROBOT_MUST_TOUCH_EE, 1);
	lgrgn += _mu.block(0, 0, NUM_OBJ_PTS*NUM_ROBOT_MUST_TOUCH_EE, 1).cwiseProduct(_cdist_obj_escape_err).sum() 
			+ _rho/2 * (_cdist_obj_escape_err.cwiseProduct(_cdist_obj_escape_err)).sum();
	_cdist_obj_escape_err.resize(NUM_OBJ_PTS, NUM_ROBOT_MUST_TOUCH_EE);
	return 1;

};


} // end namespace grasp
