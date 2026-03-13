
#include <ceres/ceres.h>
#include "core/robot_optimizer.hpp"
#include "utils/utils.hpp"

namespace grasp{


int RobotConfigOptimizer::EEObj3D() {
	if (_ee_target.empty()){
		std::cout << "[ERROR] IK_PDO::ComputeLgrgn variable _ee_target is empty." << std::endl;
		throw std::runtime_error("[ERROR]");
		return 0;
	}

	_ee_penalty = 0;
	for (auto & pair : _ee_target) {
		int i = pair.first;
		_ee_target_i = pair.second;
		// get end effector penalty
		_ee_dxyz[i][0] = _link_poses[i][0][3] - _ee_target_i[0];
		_ee_dxyz[i][1] = _link_poses[i][1][3] - _ee_target_i[1];
		_ee_dxyz[i][2] = _link_poses[i][2][3] - _ee_target_i[2];
		_ee_dist[i] = std::sqrt(_ee_dxyz[i][0] * _ee_dxyz[i][0] 
							+ _ee_dxyz[i][1] * _ee_dxyz[i][1] 
							+ _ee_dxyz[i][2] * _ee_dxyz[i][2]);
    	_ee_penalty += _ee_dist[i];
	}

	return 1;
};

int RobotConfigOptimizer::EECdistObj() {

	// Update _active_ee_position based on _active_ee_id
	// we will use this for ee objective computation, 
	// self- and obstable-collision constraints computation
	int i = 0;
	for (auto & idee : _active_ee_id) {
		_active_ee_position(i, 0) = _link_poses[idee][0][3];
		_active_ee_position(i, 1) = _link_poses[idee][1][3];
		_active_ee_position(i, 2) = _link_poses[idee][2][3];
		i++;
	}

	// Compute _cdist_mat
	DistMat(_active_ee_position, _object_position, _cdist_mat, _cdist_dx, _cdist_dy, _cdist_dz);

	// Compute _cdist_mat_err
	_cdist_mat_err = _cdist_mat - _cdist_mat_target;
	_cdist_penalty = (0.5 * _cdist_mat_err.cwiseProduct(_cdist_mat_err)).sum();

	_max_cdist_penalty = _cdist_mat_err.cwiseAbs().maxCoeff();
	
	return 1;
};

int RobotConfigOptimizer::GetMinIdx(const Eigen::MatrixXd& A,
									Eigen::MatrixXd& labelMatrix) {

    labelMatrix.resize(A.rows(), A.cols());
	double minVal;

    for (int i = 0; i < A.rows(); ++i) {
        minVal = A.row(i).minCoeff();
        for (int j = 0; j < A.cols(); ++j) {
            if (A(i, j) == minVal) {
                labelMatrix(i, j) = 1;
            } else {
				labelMatrix(i, j) = 0;
			}
        }
    }

    return 0;
}

int RobotConfigOptimizer::FormClosureConstr() {

	// form closure grasp constraint
	int i = 0;
	for (auto & idee : _must_touch_ee_id) {
		_must_touch_ee_position(i, 0) = _link_poses[idee][0][3];
		_must_touch_ee_position(i, 1) = _link_poses[idee][1][3];
		_must_touch_ee_position(i, 2) = _link_poses[idee][2][3];
		i++;
	}

	// Compute _cdist_mat when object is not escaping
	DistMat(_must_touch_ee_position, _object_position, _must_touch_cdist_mat, 
			_must_touch_cdist_dx, _must_touch_cdist_dy, _must_touch_cdist_dz);

	// Compute _cdist_mat based on _object_transition_escape
	ObjectEscapeRollout();

	DistMat(_must_touch_ee_position, _object_points_position_escape, _cdist_mat_escape, 
			_cdist_dx_escape, _cdist_dy_escape, _cdist_dz_escape);
	
	// Check if:
	// for all _cdist_mat_escape > min(_cdist_mat, threshold)
	// then the object is having a valid escape
	_cdist_obj_escape_diff = _cdist_mat_escape - _must_touch_cdist_mat.cwiseMin(_cdist_threshold);
	_mtcstt = (_must_touch_cdist_mat.array() < _cdist_threshold.array()).cast<double>();
	_coesdgtz = (_cdist_obj_escape_diff.array() > 0).cast<double>();

	// compute penalty, we check the smallest element in each row in _cdist_obj_escape_diff,
	// which means the smallest difference of each end-effector w.r.t. all of the object points
	// if all smallest differences are greater than 0, then the object is okay to escape,
	// and the grasp is not stable, we therefore penalize the robot
	if ((_cdist_obj_escape_diff.rowwise().minCoeff().array() >= 0).all()) {
		// we penalize the end effectors depending on the closest point on the object to the end effector
		Relu(_cdist_obj_escape_diff, _cdist_obj_escape_err);
		_rowwise_min_cdist = _cdist_obj_escape_err.rowwise().minCoeff();
		GetMinIdx(_cdist_obj_escape_err, _grad_rowwsie_min_cdist);
		_form_closure_penalty = _cdist_obj_escape_err.rowwise().minCoeff().sum();		
	}
	else {
		_form_closure_penalty = 0.0;
	}
		
	return 1;
};

int RobotConfigOptimizer::TouchObjective() {

	// form closure grasp constraint
	int i = 0;
	for (auto & idee : _must_touch_ee_id) {
		_must_touch_ee_position(i, 0) = _link_poses[idee][0][3];
		_must_touch_ee_position(i, 1) = _link_poses[idee][1][3];
		_must_touch_ee_position(i, 2) = _link_poses[idee][2][3];
		i++;
	}

	// Compute _cdist_mat when object is not escaping
	DistMat(_must_touch_ee_position, _object_position, _must_touch_cdist_mat, 
			_must_touch_cdist_dx, _must_touch_cdist_dy, _must_touch_cdist_dz);
	_must_touch_to_obj_min_dist = (_must_touch_cdist_mat - _cdist_threshold).rowwise().minCoeff();
	GetMinIdx(_must_touch_cdist_mat - _cdist_threshold, _grad_must_touch_to_obj_min_dist);
	_mtcgtt = ((_must_touch_cdist_mat - _cdist_threshold).array() > _cdist_threshold.array()).cast<double>();
	Relu(_must_touch_to_obj_min_dist, _touch_objective);

	return 1;
};

int RobotConfigOptimizer::ObjEscapeConstr() {
	_escape_penalty = 0;
	return 1;
};

int RobotConfigOptimizer::SelfCollisonConstr() {

	// Update _active_ee_position based on _active_ee_id
	// we will use this for ee objective computation, 
	// self- and obstable-collision constraints computation
	int i = 0;
	for (auto & idee : _active_ee_id) {
		_active_ee_position(i, 0) = _link_poses[idee][0][3];
		_active_ee_position(i, 1) = _link_poses[idee][1][3];
		_active_ee_position(i, 2) = _link_poses[idee][2][3];
		i++;
	}

	DistMat(_active_ee_position, _active_ee_position, _self_cdist_mat, 
			_self_cdist_dx, _self_cdist_dy, _self_cdist_dz);
	
	Relu(_self_cdist_mat_mini_tolerance - _self_cdist_mat, _self_cdist_mat_err);

	_self_colli_penalty = _self_cdist_mat_err.sum();
	_max_self_colli_penalty = _self_cdist_mat_err.maxCoeff();

	return 1;
};


int RobotConfigOptimizer::JointObjectCollisionConstr() {
	int i = 0;
	for (auto & idee : _active_ee_id) {
		_active_ee_position(i, 0) = _link_poses[idee][0][3];
		_active_ee_position(i, 1) = _link_poses[idee][1][3];
		_active_ee_position(i, 2) = _link_poses[idee][2][3];
		i++;
	}

	DistMat(_active_ee_position, _object_position, _colli_cdist_mat, 
			_colli_cdist_dx, _colli_cdist_dy, _colli_cdist_dz);
	
	Relu(_colli_cdist_mat_mini_tolerance - _colli_cdist_mat, _colli_cdist_mat_err);
	_colli_penalty = _colli_cdist_mat_err.sum();
	_max_colli_penalty = _colli_cdist_mat_err.maxCoeff();

	return 1;
};


int RobotConfigOptimizer::LinkObjectCollisionConstr() {
	int i = 0;
	for (auto & idgroup : _colli_link_id) {
		if (idgroup[0] == -1) {
			_colli_link_position[0](i, 0) = _base[0][3];
			_colli_link_position[0](i, 1) = _base[1][3];
			_colli_link_position[0](i, 2) = _base[2][3];
		}
		else {
			_colli_link_position[0](i, 0) = _link_poses[idgroup[0]][0][3];
			_colli_link_position[0](i, 1) = _link_poses[idgroup[0]][1][3];
			_colli_link_position[0](i, 2) = _link_poses[idgroup[0]][2][3];
		}
		_colli_link_position[1](i, 0) = _link_poses[idgroup[1]][0][3];
		_colli_link_position[1](i, 1) = _link_poses[idgroup[1]][1][3];
		_colli_link_position[1](i, 2) = _link_poses[idgroup[1]][2][3];
		i++;
	}

	DistMat(_colli_link_position[0], _object_position, _colli_link_cdist_mat0, 
			_colli_link_cdist_dx0, _colli_link_cdist_dy0, _colli_link_cdist_dz0);
	DistMat(_colli_link_position[1], _object_position, _colli_link_cdist_mat1, 
			_colli_link_cdist_dx1, _colli_link_cdist_dy1, _colli_link_cdist_dz1);

	Relu(_colli_link_cdist_mat_mini_tolerance - (_colli_link_cdist_mat0 + _colli_link_cdist_mat1), 
		_colli_link_cdist_mat_err);
	_colli_link_penalty = _colli_link_cdist_mat_err.sum();
	_max_link_colli_penalty = _colli_link_cdist_mat_err.maxCoeff();

	return 1;
};


int RobotConfigOptimizer::ComputeLgrgn() {
	
	lgrgn = 0;
	int start_id = 0;

	// self collision
	SelfCollisonConstr();
	_self_cdist_mat_err.resize(NUM_ACTIVE_EE * NUM_ACTIVE_EE, 1);
	lgrgn += _mu.block(start_id, 0, NUM_ACTIVE_EE * NUM_ACTIVE_EE, 1).cwiseProduct(_self_cdist_mat_err).sum() 
				+ _rho/2 * (_self_cdist_mat_err.cwiseProduct(_self_cdist_mat_err)).sum();
	_self_cdist_mat_err.resize(NUM_ACTIVE_EE, NUM_ACTIVE_EE);
	start_id += NUM_ACTIVE_EE * NUM_ACTIVE_EE;
	
	// obstacle collision
	JointObjectCollisionConstr();
	_colli_cdist_mat_err.resize(NUM_ACTIVE_EE * NUM_OBJ_PTS, 1);
	lgrgn += _mu.block(start_id, 0, NUM_ACTIVE_EE * NUM_OBJ_PTS, 1).cwiseProduct(_colli_cdist_mat_err).sum() 
				+ _rho/2 * (_colli_cdist_mat_err.cwiseProduct(_colli_cdist_mat_err)).sum();
	_colli_cdist_mat_err.resize(NUM_ACTIVE_EE, NUM_OBJ_PTS);
	start_id += NUM_ACTIVE_EE * NUM_OBJ_PTS;

	// link-object collision
	LinkObjectCollisionConstr();

	_colli_link_cdist_mat_err.resize(NUM_COLLISION_LINKS * NUM_OBJ_PTS, 1);
	lgrgn += _mu.block(start_id, 0, NUM_COLLISION_LINKS * NUM_OBJ_PTS, 1).cwiseProduct(_colli_link_cdist_mat_err).sum() 
				+ _rho/2 * (_colli_link_cdist_mat_err.cwiseProduct(_colli_link_cdist_mat_err)).sum();
	_colli_link_cdist_mat_err.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	start_id += NUM_COLLISION_LINKS * NUM_OBJ_PTS;

	// form  closure constraint
	FormClosureConstr();
	if (_form_closure_penalty > 0.0) {
		lgrgn += _mu.block(start_id, 0, NUM_MUST_TOUCH_EE, 1).cwiseProduct(_rowwise_min_cdist).sum() 
				+ _rho/2 * (_rowwise_min_cdist.cwiseProduct(_rowwise_min_cdist)).sum();
	}
	start_id += NUM_MUST_TOUCH_EE;

	// must touch objective
	TouchObjective();
	lgrgn += 0.5 * (_touch_objective.cwiseProduct(_touch_objective)).sum();

	return 1;
};

} // end namespace grasp
