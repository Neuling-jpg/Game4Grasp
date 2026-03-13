
#include <ceres/ceres.h>
#include "core/robot_optimizer.hpp"
#include "utils/utils.hpp"

namespace grasp{

ForwardKinematicsPDO::ForwardKinematicsPDO(){
	return ;
};

ForwardKinematicsPDO::~ForwardKinematicsPDO(){
	return ;
};

void ForwardKinematicsPDO::SetInitPoses(
	const std::vector< std::vector<std::vector<double> > >& m0s) 
{
	_m0s = m0s;
	_num_joints = _m0s.size();
};

int ForwardKinematicsPDO::Compute(const std::vector<double>& theta) {
	if (_num_joints < 0){
		std::cout << "[ERROR] FK_PDO::Compute joints and axes are not set yet!" << std::endl;
		throw std::runtime_error("[ERROR]");
		return 0;
	}

	InitLinkPoses();
	Angle2State(theta);
	FKRollout();

	return 1;
};

int ForwardKinematicsPDO::InitLinkPoses(){
	if (_num_joints < 0){
		std::cout << "[ERROR] FK_PDO::InitLinkPoses joints and axes are not set yet!" << std::endl;
		throw std::runtime_error("[ERROR]");
		return 0;
	}
	_link_poses.resize(_num_joints);
	for (int i = 0; i < _num_joints; i++){
		_link_poses[i] = MatInit(4,4,0);
	}
	return 1;
};

int ForwardKinematicsPDO::InitKinematicParams(std::vector<std::vector<double> >& jl,
	std::vector<double>& a, std::vector<double>& d, std::vector<double>& alpha, std::vector<int>& parent) {
	// Initialize the kinematic parameters for the PDO FK
	// jl: joint limits, n x 2
	// a: DH parameters 'a', n x 1
	// d: DH parameters 'd', n x 1
	// alpha: DH parameters 'alpha', n x 1
	// parent: parent link index, e.g., parent[0] = -1, parent[1] = 0, parent[2] = 1, etc.
	// Note: this function is called after the robot model is initialized, so the _num_joints is set
	//       and the joint limits, DH parameters, and parent links are set.
	if (_num_joints < 0){
		std::cout << "[ERROR] FK_PDO::InitKinematicParams joints and axes are not set yet!" << std::endl;
		throw std::runtime_error("[ERROR]");
		return 0;
	}

	_jl = jl;
	_a = a;
	_d = d;
	_alpha = alpha;
	_sub_joint_counter.resize(_num_joints);
	_sub_jl.resize(_num_joints);
	_sub_a.resize(_num_joints);
	_sub_d.resize(_num_joints);
	_sub_alpha.resize(_num_joints);
	_sub_theta.resize(_num_joints);
	_sub_DH.resize(_num_joints);
	_calpha.resize(_num_joints);
	_salpha.resize(_num_joints);
	_ctheta.resize(_num_joints);
	_stheta.resize(_num_joints);
	_joint_decision_var.resize(_num_joints);
	_dobj_dw.resize(_num_joints);
	_SR.resize(_num_joints);
	_MIN_L.resize(_num_joints);
	_MAX_L.resize(_num_joints);
	_l_vlink1.resize(_num_joints);
	_l_vlink2.resize(_num_joints);
	_dist.resize(_num_joints);
	_dist_norm.resize(_num_joints);
	_direction.resize(_num_joints);
	_DH.resize(_num_joints);

	_link_poses.resize(_num_joints);
	_sub_link_poses.resize(_num_joints);

	_parent = parent;  // set the parent links

	ROLL_MAX = M_PI; YAW_MAX = M_PI; PITCH_MAX = M_PI;
	ROLL_MIN = -M_PI; YAW_MIN = -M_PI; PITCH_MIN = -M_PI;
	_base_frame_decision_var.resize(6);
	_euler_norm.resize(6);
	_euler.resize(6);
	_base = MatEye(4);  // the base frame

	for (int i = 0; i < _num_joints; i++){

		// define what _sub_joint_counter[i] is
		int nsubj; 
		if (_jl[i][1] == _jl[i][0]) {
			nsubj = 1;
		}
		else {
			nsubj = int((_jl[i][1] - _jl[i][0] - 1e-3) / M_PI) + 1;
			if (_jl[i][0] < 0) nsubj +=1;  // if the lower limit is negative, we need one more sub-joint
		}
		
		_sub_joint_counter[i] = nsubj;

		_sub_jl[i].resize(nsubj);
		_sub_a[i].resize(nsubj);
		_sub_d[i].resize(nsubj);
		_sub_alpha[i].resize(nsubj);
		_sub_theta[i].resize(nsubj);
		_sub_DH[i].resize(nsubj);
		_calpha[i].resize(nsubj);
		_salpha[i].resize(nsubj);
		_ctheta[i].resize(nsubj);
		_stheta[i].resize(nsubj);
		_joint_decision_var[i].resize(nsubj);
		_dobj_dw[i].resize(nsubj);
		_SR[i].resize(nsubj);
		_MIN_L[i].resize(nsubj);
		_MAX_L[i].resize(nsubj);
		_l_vlink1[i].resize(nsubj);
		_l_vlink2[i].resize(nsubj);
		_dist[i].resize(nsubj);
		_dist_norm[i].resize(nsubj);
		_direction[i].resize(nsubj);

		_sub_link_poses[i].resize(nsubj);
		
		for (int j = 0; j < nsubj; j++){
			
			_joint_decision_var[i][j] = 0;
			_DH[i] = MatEye(4);
			_sub_link_poses[i][j] = MatEye(4);
			_sub_jl[i][j].resize(2);
			
			// set _l_vlink1, _l_vlink2 as 1, without loss of generality
			_l_vlink1[i][j] = 1;
			_l_vlink2[i][j] = 1;

			// set minimum and maximum distances
			if (nsubj == 1) {
				_sub_jl[i][j][0] = _jl[i][0];
				_sub_jl[i][j][1] = _jl[i][1];
			}
			else {
				if (j == nsubj - 1) {
					_sub_jl[i][j][0] = _jl[i][0];
					_sub_jl[i][j][1] = _jl[i][0];
				}
				else {
					_sub_jl[i][j][0] = 0;
					_sub_jl[i][j][1] = (_jl[i][1] - _jl[i][0]) / (nsubj - 1);
				}
			}
			
			// set sub dist limits, stroke range, and direction
			_MIN_L[i][j] = std::sqrt(_l_vlink1[i][j] * _l_vlink1[i][j] + _l_vlink2[i][j] * _l_vlink2[i][j] -
							2 * _l_vlink1[i][j] * _l_vlink2[i][j] * std::cos(_sub_jl[i][j][0]));
			_MAX_L[i][j] = std::sqrt(_l_vlink1[i][j] * _l_vlink1[i][j] + _l_vlink2[i][j] * _l_vlink2[i][j] -
							2 * _l_vlink1[i][j] * _l_vlink2[i][j] * std::cos(_sub_jl[i][j][1]));	
			_SR[i][j] = _MAX_L[i][j] - _MIN_L[i][j];
			_direction[i][j] = (_sub_jl[i][j][1] <= 0) ? -1 : 1;

			// set a, d, alpha for each sub-joint
			if (j == 0) {
				_sub_a[i][j] = _a[i];
				_sub_d[i][j] = _d[i];
				_sub_alpha[i][j] = _alpha[i];
			}
			else {
				_sub_a[i][j] = 0;
				_sub_d[i][j] = 0;
				_sub_alpha[i][j] = 0;
			}
			
			// Initialize DH matrix
			_sub_DH[i][j] = MatEye(4);
			_calpha[i][j] = std::cos(_sub_alpha[i][j]);
			_salpha[i][j] = std::sin(_sub_alpha[i][j]);
			_sub_DH[i][j][0][3] = _sub_a[i][j];
			_sub_DH[i][j][1][2] = -_salpha[i][j];
			_sub_DH[i][j][1][3] = -_sub_d[i][j] * _salpha[i][j];
			_sub_DH[i][j][2][2] = _calpha[i][j];
			_sub_DH[i][j][2][3] = _sub_d[i][j] * _calpha[i][j];

			// _SR[i] == 0 means the distance is fixed
			if (_SR[i][j] == 0) {
				_dist[i][j] = _MIN_L[i][j];
				_ctheta[i][j] = (_l_vlink1[i][j] * _l_vlink1[i][j] + _l_vlink2[i][j] * _l_vlink2[i][j] - _dist[i][j] * _dist[i][j]);
				_ctheta[i][j] = _ctheta[i][j] / (2 * _l_vlink1[i][j] * _l_vlink2[i][j]);
				_stheta[i][j] = _direction[i][j] * std::sqrt(1 - _ctheta[i][j] * _ctheta[i][j]);

				// Update DH matrix
				_sub_DH[i][j][0][0] = _ctheta[i][j];
				_sub_DH[i][j][0][1] = -_stheta[i][j];
				_sub_DH[i][j][1][0] = _stheta[i][j] * _calpha[i][j];
				_sub_DH[i][j][1][1] = _ctheta[i][j] * _calpha[i][j];
				_sub_DH[i][j][2][0] = _stheta[i][j] * _salpha[i][j];
				_sub_DH[i][j][2][1] = _ctheta[i][j] * _salpha[i][j];
			}

		}

		_link_poses[i] = MatInit(4,4,0);

	}

	num_total_sub_joints = Sum(_sub_joint_counter);

	return 1;
};

void ForwardKinematicsPDO::Angle2State(const std::vector<double>& theta) {
	for (int i = 0; i < _num_joints; i++) {
		for (int j = 0; j < _sub_joint_counter[i]; j++) {
			if (_SR[i][j] != 0) {
				
				if(_sub_joint_counter[i]==1) _sub_theta[i][j] = theta[i] - _jl[i][0];
				else _sub_theta[i][j] = (theta[i] - _jl[i][0]) / (_sub_joint_counter[i] - 1);
				
				_ctheta[i][j] = std::cos(_sub_theta[i][j]);
				_dist[i][j] = _l_vlink1[i][j] * _l_vlink1[i][j] + _l_vlink2[i][j] * _l_vlink2[i][j] 
								- 2 * _l_vlink1[i][j] * _l_vlink2[i][j] * _ctheta[i][j];
				_dist[i][j] = std::sqrt(_dist[i][j]);
				_dist_norm[i][j] = (_dist[i][j] - _MIN_L[i][j]) / _SR[i][j];
				_joint_decision_var[i][j] = InvSigmoid(_dist_norm[i][j]);
			}
			else {
				_sub_theta[i][j] = _jl[i][0];
			}	
		}
	}
};

void ForwardKinematicsPDO::FKRollout() {
	// update base frame
	for (int i=0; i<6; ++i) _euler_norm[i] = Sigmoid(_base_frame_decision_var[i]);
	
	_euler[0] = ROLL_MIN + (ROLL_MAX - ROLL_MIN) * _euler_norm[0];  // roll
	_euler[1] = YAW_MIN + (YAW_MAX - YAW_MIN) * _euler_norm[1];  // yaw 
	_euler[2] = PITCH_MIN + (PITCH_MAX - PITCH_MIN) * _euler_norm[2];  // pitch
	_euler[3] = -0.5 + 1 * _euler_norm[3];
	_euler[4] = -0.5 + 1 * _euler_norm[4];
	_euler[5] = -0.5 + 1 * _euler_norm[5];
	Euler2Mat(_euler, _base);

	// update link frames based on joint angles
	for (int i = 0; i < _parent.size(); ++i) {
		int i_parent = _parent[i];
		
		for (int j = 0; j < _sub_joint_counter[i]; j++){
			
			if (_SR[i][j] != 0) {
				// Calculate distance
				_dist_norm[i][j] = Sigmoid(_joint_decision_var[i][j]);
				_dist[i][j] = _SR[i][j] * _dist_norm[i][j] + _MIN_L[i][j];

				// Calculate ctheta, stheta
				_ctheta[i][j] = (_l_vlink1[i][j] * _l_vlink1[i][j] + _l_vlink2[i][j] * _l_vlink2[i][j] - _dist[i][j] * _dist[i][j]);
				_ctheta[i][j] = _ctheta[i][j] / (2 * _l_vlink1[i][j] * _l_vlink2[i][j]);
				_stheta[i][j] = _direction[i][j] * std::sqrt(1 - _ctheta[i][j] * _ctheta[i][j]);

				// Update DH matrix
				_sub_DH[i][j][0][0] = _ctheta[i][j];
				_sub_DH[i][j][0][1] = -_stheta[i][j];
				_sub_DH[i][j][1][0] = _stheta[i][j] * _calpha[i][j];
				_sub_DH[i][j][1][1] = _ctheta[i][j] * _calpha[i][j];
				_sub_DH[i][j][2][0] = _stheta[i][j] * _salpha[i][j];
				_sub_DH[i][j][2][1] = _ctheta[i][j] * _salpha[i][j];
			}

			// Update link poses
			if (i_parent == -1 && j == 0) {
				_sub_link_poses[i][j] = _base * _sub_DH[i][j];
				_DH[i] = _sub_DH[i][j];
			}
			else if (j == 0) {
				_sub_link_poses[i][j] = _sub_link_poses[i_parent][_sub_joint_counter[i_parent]-1] * _sub_DH[i][j];
				_DH[i] = _sub_DH[i][j];
			} 
			else {
				_sub_link_poses[i][j] = _sub_link_poses[i][j-1] * _sub_DH[i][j];
				_DH[i] = _DH[i] * _sub_DH[i][j];
			}
		}

		_link_poses[i] = _sub_link_poses[i][_sub_joint_counter[i]-1];
	}

};

std::vector<std::vector<double> > ForwardKinematicsPDO::GetLinkPose(int k){
	std::vector<std::vector<double> > out;
	if (k < 0 || k >= _num_joints){
		std::cout << "[ERROR] FK_PDO::GetLinkPose try to get the pose of a link that does not exists! k=" << k << std::endl;
		throw std::runtime_error("[ERROR]");
		return out;
	}
	return _link_poses[k];
};

int RobotConfigOptimizer::ObjectEscapeRollout() {
    // Precompute trigonometric values
    obj_cr = cos(_object_rotation_euler_escape(0));
    obj_sr = sin(_object_rotation_euler_escape(0));
	obj_cy = cos(_object_rotation_euler_escape(1));
    obj_sy = sin(_object_rotation_euler_escape(1));
    obj_cp = cos(_object_rotation_euler_escape(2));
    obj_sp = sin(_object_rotation_euler_escape(2));
    
    // Allocate 3x3 matrix
    std::vector<std::vector<double>> rotationMatrix(3, std::vector<double>(3));

    // Calculate rotation matrix elements
    rotationMatrix[0][0] = obj_cy * obj_cp;
    rotationMatrix[0][1] = obj_cy * obj_sp * obj_sr - obj_sy * obj_cr;
    rotationMatrix[0][2] = obj_cy * obj_sp * obj_cr + obj_sy * obj_sr;

    rotationMatrix[1][0] = obj_sy * obj_cp;
    rotationMatrix[1][1] = obj_sy * obj_sp * obj_sr + obj_cy * obj_cr;
    rotationMatrix[1][2] = obj_sy * obj_sp * obj_cr - obj_cy * obj_sr;

    rotationMatrix[2][0] = -obj_sp;
    rotationMatrix[2][1] = obj_cp * obj_sr;
    rotationMatrix[2][2] = obj_cp * obj_cr;

	_object_rotation_mat_escape << obj_cy * obj_cp, obj_cy * obj_sp * obj_sr - obj_sy * obj_cr, obj_cy * obj_sp * obj_cr + obj_sy * obj_sr,
								   obj_sy * obj_cp, obj_sy * obj_sp * obj_sr + obj_cy * obj_cr, obj_sy * obj_sp * obj_cr - obj_cy * obj_sr,
								   -obj_sp        , obj_cp * obj_sr                           , obj_cp * obj_cr                           ;

	// forward rollout
	_object_points_position_escape = _object_position * _object_rotation_mat_escape 
										+ _object_translation_escape.transpose().replicate(_object_position.rows(), 1);

    return 1;
}

} // end namespace grasp
