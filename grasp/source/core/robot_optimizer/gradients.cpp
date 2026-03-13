
#include <ceres/ceres.h>
#include "core/robot_optimizer.hpp"
#include "utils/utils.hpp"

namespace grasp{

int RobotConfigOptimizer::ComputeJacob() {

	int i, start_id = 0;

	///////////////////////////////////////
	// derivatives w.r.t. self-collision
	///////////////////////////////////////

	_self_cdist_mat.array() += EPSILON;
	_dpenalty_deex = -_self_cdist_dx.cwiseQuotient(_self_cdist_mat);
	_dpenalty_deey = -_self_cdist_dy.cwiseQuotient(_self_cdist_mat);
	_dpenalty_deez = -_self_cdist_dz.cwiseQuotient(_self_cdist_mat);

	// resize to fit the dimension of mu
	_self_cdist_mat_err_greater_than_zero = (_self_cdist_mat_err.array() > 0.0).cast<double>();
	_self_cdist_mat_err_greater_than_zero.resize(NUM_ACTIVE_EE * NUM_ACTIVE_EE, 1);
	_self_cdist_mat_err.resize(NUM_ACTIVE_EE * NUM_ACTIVE_EE, 1);
	_dpenalty_deex.resize(NUM_ACTIVE_EE * NUM_ACTIVE_EE, 1);
	_dpenalty_deey.resize(NUM_ACTIVE_EE * NUM_ACTIVE_EE, 1);
	_dpenalty_deez.resize(NUM_ACTIVE_EE * NUM_ACTIVE_EE, 1);

	// compute derivatives of lagrangian w.r.t. end-effector (x, y, z)
	slack_var = _self_cdist_mat_err_greater_than_zero
					.cwiseProduct((_mu.block(start_id, 0, NUM_ACTIVE_EE * NUM_ACTIVE_EE, 1) 
											+ _rho * _self_cdist_mat_err));
	_dlgrgn_deex = slack_var.cwiseProduct(_dpenalty_deex);
	_dlgrgn_deey = slack_var.cwiseProduct(_dpenalty_deey);
	_dlgrgn_deez = slack_var.cwiseProduct(_dpenalty_deez);

	// resize them back
	_self_cdist_mat_err.resize(NUM_ACTIVE_EE, NUM_ACTIVE_EE);
	_dlgrgn_deex.resize(NUM_ACTIVE_EE, NUM_ACTIVE_EE);
	_dlgrgn_deey.resize(NUM_ACTIVE_EE, NUM_ACTIVE_EE);
	_dlgrgn_deez.resize(NUM_ACTIVE_EE, NUM_ACTIVE_EE);
	
	// plug the derivatives into robot model
	i = 0;
	for (auto & idee : _active_ee_id) {
		_dLgrgn_dpose[idee].back()[0][3] += _dlgrgn_deex.rowwise().sum()(i);
		_dLgrgn_dpose[idee].back()[1][3] += _dlgrgn_deey.rowwise().sum()(i);
		_dLgrgn_dpose[idee].back()[2][3] += _dlgrgn_deez.rowwise().sum()(i);
		i++;
	}

	start_id += NUM_ACTIVE_EE * NUM_ACTIVE_EE;

	///////////////////////////////////////
	// derivatives w.r.t. joint-object collision
	///////////////////////////////////////

	_colli_cdist_mat.array() += EPSILON;
	_dpenalty_deex = -_colli_cdist_dx.cwiseQuotient(_colli_cdist_mat);
	_dpenalty_deey = -_colli_cdist_dy.cwiseQuotient(_colli_cdist_mat);
	_dpenalty_deez = -_colli_cdist_dz.cwiseQuotient(_colli_cdist_mat);

	// resize to fit the dimension of mu
	_colli_cdist_mat_err_greater_than_zero = (_colli_cdist_mat_err.array() > 0.0).cast<double>();
	_colli_cdist_mat_err_greater_than_zero.resize(NUM_ACTIVE_EE * NUM_OBJ_PTS, 1);
	_colli_cdist_mat_err.resize(NUM_ACTIVE_EE * NUM_OBJ_PTS, 1);
	_dpenalty_deex.resize(NUM_ACTIVE_EE * NUM_OBJ_PTS, 1);
	_dpenalty_deey.resize(NUM_ACTIVE_EE * NUM_OBJ_PTS, 1);
	_dpenalty_deez.resize(NUM_ACTIVE_EE * NUM_OBJ_PTS, 1);

	// compute derivatives of lagrangian w.r.t. end-effector (x, y, z)
	slack_var = _colli_cdist_mat_err_greater_than_zero
					.cwiseProduct((_mu.block(start_id, 0, NUM_ACTIVE_EE * NUM_OBJ_PTS, 1) 
											+ _rho * _colli_cdist_mat_err));
	_dlgrgn_deex = slack_var.cwiseProduct(_dpenalty_deex);
	_dlgrgn_deey = slack_var.cwiseProduct(_dpenalty_deey);
	_dlgrgn_deez = slack_var.cwiseProduct(_dpenalty_deez);

	// resize them back
	_colli_cdist_mat_err.resize(NUM_ACTIVE_EE, NUM_OBJ_PTS);
	_dlgrgn_deex.resize(NUM_ACTIVE_EE, NUM_OBJ_PTS);
	_dlgrgn_deey.resize(NUM_ACTIVE_EE, NUM_OBJ_PTS);
	_dlgrgn_deez.resize(NUM_ACTIVE_EE, NUM_OBJ_PTS);

	// plug the derivatives into robot model
	i = 0;
	for (auto & idee : _active_ee_id) {
		_dLgrgn_dpose[idee].back()[0][3] += _dlgrgn_deex.rowwise().sum()(i);
		_dLgrgn_dpose[idee].back()[1][3] += _dlgrgn_deey.rowwise().sum()(i);
		_dLgrgn_dpose[idee].back()[2][3] += _dlgrgn_deez.rowwise().sum()(i);
		i++;
	}

	start_id += NUM_ACTIVE_EE * NUM_OBJ_PTS;

	///////////////////////////////////////
	// derivatives w.r.t. link-object collision
	///////////////////////////////////////

	_colli_link_cdist_mat0.array() += EPSILON; _colli_link_cdist_mat1.array() += EPSILON;
	_dpenalty_deex0 = -_colli_link_cdist_dx0.cwiseQuotient(_colli_link_cdist_mat0);
	_dpenalty_deey0 = -_colli_link_cdist_dy0.cwiseQuotient(_colli_link_cdist_mat0);
	_dpenalty_deez0 = -_colli_link_cdist_dz0.cwiseQuotient(_colli_link_cdist_mat0);
	_dpenalty_deex1 = -_colli_link_cdist_dx1.cwiseQuotient(_colli_link_cdist_mat1);
	_dpenalty_deey1 = -_colli_link_cdist_dy1.cwiseQuotient(_colli_link_cdist_mat1);
	_dpenalty_deez1 = -_colli_link_cdist_dz1.cwiseQuotient(_colli_link_cdist_mat1);

	// resize to fit the dimension of mu
	_colli_link_cdist_mat_err_greater_than_zero = (_colli_link_cdist_mat_err.array() > 0.0).cast<double>();
	_colli_link_cdist_mat_err_greater_than_zero.resize(NUM_COLLISION_LINKS * NUM_OBJ_PTS, 1);
	_colli_link_cdist_mat_err.resize(NUM_COLLISION_LINKS * NUM_OBJ_PTS, 1);
	_dpenalty_deex0.resize(NUM_COLLISION_LINKS * NUM_OBJ_PTS, 1);
	_dpenalty_deey0.resize(NUM_COLLISION_LINKS * NUM_OBJ_PTS, 1);
	_dpenalty_deez0.resize(NUM_COLLISION_LINKS * NUM_OBJ_PTS, 1);
	_dpenalty_deex1.resize(NUM_COLLISION_LINKS * NUM_OBJ_PTS, 1);
	_dpenalty_deey1.resize(NUM_COLLISION_LINKS * NUM_OBJ_PTS, 1);
	_dpenalty_deez1.resize(NUM_COLLISION_LINKS * NUM_OBJ_PTS, 1);

	// compute derivatives of lagrangian w.r.t. end-effector (x, y, z)
	slack_var = _colli_link_cdist_mat_err_greater_than_zero
					.cwiseProduct((_mu.block(start_id, 0, NUM_COLLISION_LINKS * NUM_OBJ_PTS, 1) 
											+ _rho * _colli_link_cdist_mat_err));
	_dlgrgn_deex0 = slack_var.cwiseProduct(_dpenalty_deex0);
	_dlgrgn_deey0 = slack_var.cwiseProduct(_dpenalty_deey0);
	_dlgrgn_deez0 = slack_var.cwiseProduct(_dpenalty_deez0);
	_dlgrgn_deex1 = slack_var.cwiseProduct(_dpenalty_deex1);
	_dlgrgn_deey1 = slack_var.cwiseProduct(_dpenalty_deey1);
	_dlgrgn_deez1 = slack_var.cwiseProduct(_dpenalty_deez1);

	// resize them back
	_colli_link_cdist_mat_err.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_dlgrgn_deex0.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_dlgrgn_deey0.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_dlgrgn_deez0.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_dlgrgn_deex1.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_dlgrgn_deey1.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);
	_dlgrgn_deez1.resize(NUM_COLLISION_LINKS, NUM_OBJ_PTS);

	// plug the derivatives into robot model
	i = 0;
	for (auto & idee : _colli_link_id) {
		if (idee[0] != -1) {
			_dLgrgn_dpose[idee[0]].back()[0][3] += _dlgrgn_deex0.rowwise().sum()(i);
			_dLgrgn_dpose[idee[0]].back()[1][3] += _dlgrgn_deey0.rowwise().sum()(i);
			_dLgrgn_dpose[idee[0]].back()[2][3] += _dlgrgn_deez0.rowwise().sum()(i);
		}
		else {
			_dLgrgn_dbase[0][3] += _dlgrgn_deex0.rowwise().sum()(i);
			_dLgrgn_dbase[1][3] += _dlgrgn_deey0.rowwise().sum()(i);
			_dLgrgn_dbase[2][3] += _dlgrgn_deez0.rowwise().sum()(i);
		}
		if (idee[1] != -1) {
			_dLgrgn_dpose[idee[1]].back()[0][3] += _dlgrgn_deex1.rowwise().sum()(i);
			_dLgrgn_dpose[idee[1]].back()[1][3] += _dlgrgn_deey1.rowwise().sum()(i);
			_dLgrgn_dpose[idee[1]].back()[2][3] += _dlgrgn_deez1.rowwise().sum()(i);
		}
		else {
			_dLgrgn_dbase[0][3] += _dlgrgn_deex1.rowwise().sum()(i);
			_dLgrgn_dbase[1][3] += _dlgrgn_deey1.rowwise().sum()(i);
			_dLgrgn_dbase[2][3] += _dlgrgn_deez1.rowwise().sum()(i);
		}
		i++;
	}

	start_id += NUM_COLLISION_LINKS * NUM_OBJ_PTS;

	///////////////////////////////////////
	// derivatives w.r.t. form closure
	///////////////////////////////////////

	if (_form_closure_penalty > 0.0) {
		
		_cdist_mat_escape.array() += EPSILON; _must_touch_cdist_mat.array() += EPSILON;
		_dpenalty_deex = _cdist_dx_escape.cwiseQuotient(_cdist_mat_escape);
		_dpenalty_deey = _cdist_dy_escape.cwiseQuotient(_cdist_mat_escape);
		_dpenalty_deez = _cdist_dz_escape.cwiseQuotient(_cdist_mat_escape);
		// _mtcstt is short for _must_touch_cdist_smaller_than_threshold
		_dpenalty_deex -= _mtcstt.cwiseProduct(_must_touch_cdist_dx).cwiseQuotient(_must_touch_cdist_mat);
		_dpenalty_deey -= _mtcstt.cwiseProduct(_must_touch_cdist_dy).cwiseQuotient(_must_touch_cdist_mat);
		_dpenalty_deez -= _mtcstt.cwiseProduct(_must_touch_cdist_dz).cwiseQuotient(_must_touch_cdist_mat);

		_dpenalty_deex = _dpenalty_deex.cwiseProduct(_coesdgtz);
		_dpenalty_deey = _dpenalty_deey.cwiseProduct(_coesdgtz);
		_dpenalty_deez = _dpenalty_deez.cwiseProduct(_coesdgtz);

		_dpenalty_deex_rmin_filtered = _dpenalty_deex.cwiseProduct(_grad_rowwsie_min_cdist).rowwise().sum();
		_dpenalty_deey_rmin_filtered = _dpenalty_deey.cwiseProduct(_grad_rowwsie_min_cdist).rowwise().sum();
		_dpenalty_deez_rmin_filtered = _dpenalty_deez.cwiseProduct(_grad_rowwsie_min_cdist).rowwise().sum();

		slack_var = _mu.block(start_id, 0, NUM_MUST_TOUCH_EE, 1) + _rho * _rowwise_min_cdist;

		_dlgrgn_deex = slack_var.cwiseProduct(_dpenalty_deex_rmin_filtered);
		_dlgrgn_deey = slack_var.cwiseProduct(_dpenalty_deey_rmin_filtered);
		_dlgrgn_deez = slack_var.cwiseProduct(_dpenalty_deez_rmin_filtered);

		// plug the derivatives into robot model
		i = 0;
		for (auto & idee : _must_touch_ee_id) {
			_dLgrgn_dpose[idee].back()[0][3] += _dlgrgn_deex(i);
			_dLgrgn_dpose[idee].back()[1][3] += _dlgrgn_deey(i);
			_dLgrgn_dpose[idee].back()[2][3] += _dlgrgn_deez(i);
			i++;
		}
	}
	
	///////////////////////////////////////
	// derivatives w.r.t. touch objective
	///////////////////////////////////////

	_dpenalty_deex = _must_touch_cdist_dx.cwiseProduct(_mtcgtt);
	_dpenalty_deey = _must_touch_cdist_dy.cwiseProduct(_mtcgtt);
	_dpenalty_deez = _must_touch_cdist_dz.cwiseProduct(_mtcgtt);

	_dlgrgn_deex = _dpenalty_deex.cwiseProduct(_grad_must_touch_to_obj_min_dist).rowwise().sum();
	_dlgrgn_deey = _dpenalty_deey.cwiseProduct(_grad_must_touch_to_obj_min_dist).rowwise().sum();
	_dlgrgn_deez = _dpenalty_deez.cwiseProduct(_grad_must_touch_to_obj_min_dist).rowwise().sum();
	
	// plug the derivatives into robot model
	i = 0;
	for (auto & idee : _must_touch_ee_id) {
		_dLgrgn_dpose[idee].back()[0][3] += _dlgrgn_deex(i);
		_dLgrgn_dpose[idee].back()[1][3] += _dlgrgn_deey(i);
		_dLgrgn_dpose[idee].back()[2][3] += _dlgrgn_deez(i);
		i++;
	}
	
	// chain rule gradient computation
	for (int i = _num_joints - 1; i >= 0; --i) {
		int i_parent = _parent[i];
		// propagate gradients from this sub-joint to the previous one
		for (int j = _sub_joint_counter[i] - 1; j > 0; --j) GradPropagate(i, j, i, j - 1);  // when j > 0
		// when j = 0, propagate to parent joint
		GradPropagate(i, 0, i_parent, _sub_joint_counter[i_parent]-1); 
	}

	BaseGradCompute();

	return 1;
};

int RobotConfigOptimizer::GradPropagate(const int& i, const int& j, const int& i_prv, const int& j_prv) {
	if (i_prv == -1) {
		_dLgrgn_dbase = _dLgrgn_dpose[i][j] * MatTranspose(_sub_DH[i][j]);
		if (_SR[i][j] == 0) return 1;
		// _dLgrgn_dDH[i][j] = MatEye(4) * _dLgrgn_dpose[i][j];
		_dLgrgn_dDH[i][j] = MatTranspose(_base) * _dLgrgn_dpose[i][j];
	}
	else {
		_dLgrgn_dpose[i_prv][j_prv] += _dLgrgn_dpose[i][j] * MatTranspose(_sub_DH[i][j]);
		if (_SR[i][j] == 0) return 1;
		_dLgrgn_dDH[i][j] = MatTranspose(_sub_link_poses[i_prv][j_prv]) * _dLgrgn_dpose[i][j];
	}

	_dctheta_ddist[i][j] = -_dist[i][j] / (_l_vlink1[i][j] * _l_vlink2[i][j]);
	_ddistnorm_dw[i][j] = _dist_norm[i][j] * (1 - _dist_norm[i][j]);
    _ddist_dw[i][j] = _SR[i][j] * _ddistnorm_dw[i][j];
	_dstheta_dctheta[i][j] = -_ctheta[i][j] / (_stheta[i][j] + EPSILON);

	_dDH_dctheta[i][j][0][0] = 1;
	_dDH_dctheta[i][j][0][1] = -_dstheta_dctheta[i][j];
	_dDH_dctheta[i][j][1][0] = _calpha[i][j] * _dstheta_dctheta[i][j];
	_dDH_dctheta[i][j][1][1] = _calpha[i][j];
	_dDH_dctheta[i][j][2][0] = _salpha[i][j] * _dstheta_dctheta[i][j];
	_dDH_dctheta[i][j][2][1] = _salpha[i][j];

	_dLgrgn_dctheta[i][j] = Sum(cwiseMatMul(_dLgrgn_dDH[i][j], _dDH_dctheta[i][j]));

	_dLgrgn_ddist[i][j] = _dLgrgn_dctheta[i][j] * _dctheta_ddist[i][j];

	_dobj_dw[i][j] += _dLgrgn_ddist[i][j] * _ddist_dw[i][j];

	return 1;
};

int RobotConfigOptimizer::BaseGradCompute() {
	// Precompute trig terms
    double cr = cos(_euler[0]), sr = sin(_euler[0]);
    double cy = cos(_euler[1]), sy = sin(_euler[1]);
    double cp = cos(_euler[2]), sp = sin(_euler[2]);

	std::vector<std::vector<double>> _dRbase_droll = {
														{0, cy*sp*cr + sy*sr, -cy*sp*sr + sy*cr},
														{0, sy*sp*cr - cy*sr, -sy*sp*sr - cy*cr},
														{0, cp*cr,           -cp*sr,           },
														} ;
	std::vector<std::vector<double>> _dRbase_dpitch = {
														{-cy*sp, cy*cp*sr, cy*cp*cr},
														{-sy*sp, sy*cp*sr, sy*cp*cr},
														{-cp,    -sp*sr,   -sp*cr  },
														} ;
	std::vector<std::vector<double>> _dRbase_dyaw = {
														{-sy*cp, -sy*sp*sr - cy*cr, -sy*sp*cr + cy*sr},
														{cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr },
														{0,      0,                0,                },
														} ;
	_grad_base_euler_angle[0] = Sum(cwiseMatMul(GetSubMat(_dLgrgn_dbase, 0, 3, 0, 3), _dRbase_droll));
	_grad_base_euler_angle[1] = Sum(cwiseMatMul(GetSubMat(_dLgrgn_dbase, 0, 3, 0, 3), _dRbase_dyaw));
	_grad_base_euler_angle[2] = Sum(cwiseMatMul(GetSubMat(_dLgrgn_dbase, 0, 3, 0, 3), _dRbase_dpitch));

	_grad_base_euler[0] = _grad_base_euler_angle[0] * (ROLL_MAX - ROLL_MIN) * _euler_norm[0] * (1 - _euler_norm[0]);
	_grad_base_euler[1] = _grad_base_euler_angle[1] * (YAW_MAX - YAW_MIN) * _euler_norm[1] * (1 - _euler_norm[1]);
	_grad_base_euler[2] = _grad_base_euler_angle[2] * (PITCH_MAX - PITCH_MIN) * _euler_norm[2] * (1 - _euler_norm[2]);

	if (!fix_base_translation) {
		_grad_base_euler[3] = _dLgrgn_dbase[0][3] * 1 * _euler_norm[3] * (1 - _euler_norm[3]);
		_grad_base_euler[4] = _dLgrgn_dbase[1][3] * 1 * _euler_norm[4] * (1 - _euler_norm[4]);
		_grad_base_euler[5] = _dLgrgn_dbase[2][3] * 1 * _euler_norm[5] * (1 - _euler_norm[5]);

	}
	else {
		_grad_base_euler[3] = 0.0; _grad_base_euler[4] = 0.0; _grad_base_euler[5] = 0.0;
	}
	
	return 1;
}


} // end namespace grasp
