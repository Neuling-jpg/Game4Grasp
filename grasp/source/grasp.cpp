#include <set>
#include <limits>
#include <numbers>
#include <string_view>
#include <vector>
#include <algorithm>

#include "grasp.hpp"
#include "kinematics_models/robot_registry.hpp"

namespace grasp{

GraspOpt::GraspOpt(const std::string_view& robot_name, const bool& verbose) {
	_verbose = verbose;
	_fk_ptr = new ForwardKinematicsPDO(); 	
	_robot_solver = new RobotConfigOptimizer();
	_obj_solver = new ObjectPoseOptimizer();
	_robot_name = robot_name;
	auto model = grasp::RobotRegistry::instance().make(_robot_name);
  	_init(model);
};

GraspOpt::~GraspOpt() {
	delete _fk_ptr;
	delete _robot_solver;
	delete _obj_solver;
	return;
};	

void GraspOpt::_init(const KinematicModel& km) {
  
  // register kinematics chain
  num_joints = km.num_joints;
  _a = km.a; _d = km.d; _alpha = km.alpha; 
  _parent = km.parent; _link_names = km.name;
  _m0s = km.m0s;

  joint_limits.resize(num_joints);
  _theta_ini.resize(num_joints);
  solid_joint_id = {}; solid_joint_rad = {};
  solid_link_id = {}; solid_link_rad = {};
  for (int i=0;i<num_joints;++i) {
    joint_limits[i].resize(2);
    joint_limits[i][0] = km.limits[i][0];
    joint_limits[i][1] = km.limits[i][1];
    _theta_ini[i] = km.ini_angles[i];
    if (km.jrad[i]>0) {
      solid_joint_id.emplace_back(i);
      solid_joint_rad.emplace_back(km.jrad[i]);
    }
    if (km.lrad[i]>0) {
      std::vector<int> link_two_end_id = {km.parent[i], i};
      solid_link_id.emplace_back(link_two_end_id);
      solid_link_rad.emplace_back(km.lrad[i]);
    }
    if (km.trad[i]>0) {
      must_touch_obj_ee_id.emplace_back(i);
      must_touch_obj_ee_rad.emplace_back(km.trad[i]);
    }
  }
  
  _fk_ptr->SetInitPoses(_m0s);
  _robot_solver->SetInitPoses(_m0s);
  _fk_ptr->InitKinematicParams(joint_limits, _a, _d, _alpha, _parent);
  _robot_solver->InitKinematicParams(joint_limits, _a, _d, _alpha, _parent);

}

std::vector<std::string> GraspOpt::GetLinkNames() {return _link_names;} ;

void GraspOpt::UpdateBaseFrame(const std::vector<std::vector<double>>& base) {
	_fk_ptr->_base = base;
	_robot_solver->_base = base;
} ;

void GraspOpt::UpdateBaseEuler6D(const std::vector<double>& euler) {
	_fk_ptr->_base_frame_decision_var.resize(6);

	// rotation
	std::vector<std::vector<double>> euler_lims;
	euler_lims = {
		{_robot_solver->ROLL_MIN, _robot_solver->ROLL_MAX},
		{_robot_solver->YAW_MIN, _robot_solver->YAW_MAX},
		{_robot_solver->PITCH_MIN, _robot_solver->PITCH_MAX},
	} ;
	for (int i = 0; i < 3; ++i) {
		double euler_norm = (euler[i] - euler_lims[i][0]) / (euler_lims[i][1] - euler_lims[i][0]);
		_fk_ptr->_base_frame_decision_var[i] = InvSigmoid(euler_norm);
		_robot_solver->_base_frame_decision_var[i] = InvSigmoid(euler_norm);
	}
	// translation
	for (int i = 3; i < 6; ++i) {
		double euler_norm = (euler[i] + 0.5) / 1;
		_fk_ptr->_base_frame_decision_var[i] = InvSigmoid(euler_norm);
		_robot_solver->_base_frame_decision_var[i] = InvSigmoid(euler_norm);
	}
} ;


void GraspOpt::GetBaseFrame(std::vector<std::vector<double>>& base) {
	// base = _fk_ptr->_base;
	base = _robot_solver->_base;
} ;

std::vector<std::vector<double>> GraspOpt::GetBaseFrame() {
	// base = _fk_ptr->_base;
	return _robot_solver->_base;
} ;

void GraspOpt::SetCdistParams(const std::vector<int>& solid_joint_id,
								const std::vector<double>& solid_joint_rad,
								const std::vector<std::vector<int>>& solid_link_id,
								const std::vector<double>& solid_link_rad,
								const std::vector< std::vector<double> >& object_points,
								const std::vector< std::vector<double> >& cdist_target) {
	_robot_solver->SetCdistComputeParams(solid_joint_id, solid_joint_rad, 
											solid_link_id, solid_link_rad, 
											object_points, cdist_target);
} ;

void GraspOpt::ComputeFK(const std::vector<double>& theta) {
	_fk_ptr->Compute(theta);
} ;


std::vector<std::vector<double> > GraspOpt::GetLinkPose(int k) {
	return _robot_solver->GetLinkPose(k);
} ;

void GraspOpt::ComputeIK(const std::vector<double>& theta_ini, 
								std::vector<double>& theta_solution,
								const std::map<int, std::vector<double>>& ee_target_position,
								const std::vector<std::vector<double>>& points) {
	
	_robot_solver -> SetEETargetPosition(ee_target_position);

	if(_verbose) std::cout << "[ComputeIK] SetEETargetPosition" << std::endl;

	_robot_solver -> SetObstacles(points, _colli_object_idx, _colli_object_radius);

	if(_verbose) std::cout << "[ComputeIK] SetObstacles" << std::endl;

	_obj_solver -> InitParams();

	if(_verbose) std::cout << "[ComputeIK] ObjPoseSolver InitParams" << std::endl;
	
	// _robot_solver->Compute(theta_ini);
	GraspSolver(_robot_solver, _obj_solver, theta_ini, false, _verbose);

	if(_verbose) std::cout << "[ComputeIK] GraspSolver" << std::endl;
	
	_fk_ptr->Compute(_robot_solver->joint_solution);
	
	if(_verbose) std::cout << "[ComputeIK] _fk_ptr->Compute" << std::endl;
	
	theta_solution = _robot_solver->joint_solution;
	
	ee_penalty = _robot_solver->_ee_penalty;
	colli_penalty = _robot_solver->_colli_penalty;
	penalty = _robot_solver->_ee_penalty + _robot_solver->_colli_penalty;
} ;

void GraspOpt::ComputeIKCdist(std::vector<double>& theta_solution,
								const std::vector<double>& theta_ini, 
								const std::vector<std::vector<int>>& solid_link_id,
								const std::vector<double>& solid_link_rad,
								const std::vector<std::vector<double>>& object_pts,
								const std::vector<std::vector<double>>& cdist_target,
								const std::vector<std::vector<double>>& obstacle_pts) {
	
	_robot_solver -> SetCdistComputeParams(solid_joint_id, solid_joint_rad, 
											solid_link_id, solid_link_rad, 
											object_pts, cdist_target);
	_robot_solver -> SetObstacles(obstacle_pts, _colli_object_idx, _colli_object_radius);

	if(_verbose) std::cout << "[ComputeIK] Initialize" << std::endl;
	
	// _robot_solver->Compute(theta_ini);
	GraspSolver(_robot_solver, _obj_solver, theta_ini, true, _verbose);

	if(_verbose) std::cout << "[ComputeIK] GraspSolver" << std::endl;

	theta_solution = _robot_solver->joint_solution;
	
	ee_penalty = _robot_solver->_ee_penalty + _robot_solver->_max_cdist_penalty;
	colli_penalty = _robot_solver->_colli_penalty + _robot_solver->_max_self_colli_penalty;
	penalty = _robot_solver->_ee_penalty + _robot_solver->_colli_penalty;
} ;

Eigen::Matrix3d GraspOpt::getMajorAxis(
    const Eigen::MatrixXd& pointcloud)
{
    int N = pointcloud.rows();

	Eigen::Vector3d centroid = pointcloud.colwise().mean();
    Eigen::MatrixXd centered = pointcloud.rowwise() - centroid.transpose();
    Eigen::Matrix3d cov = (centered.transpose() * centered) / double(N);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Vector3d eigvals = es.eigenvalues();
    Eigen::Matrix3d eigvecs = es.eigenvectors();
    std::vector<int> idx = {0, 1, 2};
    std::sort(idx.begin(), idx.end(), [&](int i, int j) { return eigvals(i) > eigvals(j); });    
	Eigen::Matrix3d R;
    R.col(0) = eigvecs.col(idx[0]);
    R.col(1) = eigvecs.col(idx[1]);
    R.col(2) = eigvecs.col(idx[2]);
    if (R.determinant() < 0)
        R.col(2) = -R.col(2);    
	
	return R;
};

void GraspOpt::AdjustBase(RobotConfigOptimizer* robot_solver_ptr, 
							ObjectPoseOptimizer* object_solver_ptr ,
							const std::vector<double>& theta, 
							const bool& use_cdist, 
							const bool& _verbose,
							const std::vector<std::vector<double>>& base_ini) {
	
	// initialization
	robot_solver_ptr -> InitLinkPoses();

	robot_solver_ptr -> Angle2State(theta);
	robot_solver_ptr -> _object_rotation_euler_escape << 0, 0, 0;
	robot_solver_ptr -> _object_translation_escape << 0, 0, 0;
	
	// give robot base a good start
	robot_solver_ptr -> InitGradValue();
	robot_solver_ptr -> InitLgrgnWeight();
	robot_solver_ptr -> FKRollout();
	robot_solver_ptr -> ComputeLgrgn();

	Eigen::MatrixXd new_base;

	Eigen::MatrixXd robot_cloud = robot_solver_ptr -> _must_touch_ee_position;
	Eigen::MatrixXd object_cloud = robot_solver_ptr -> _object_position;

	// Compute centroids
    // Eigen::RowVector3d centroid_robot = robot_cloud.colwise().mean();
    Eigen::RowVector3d centroid_robot = (robot_cloud.colwise().minCoeff() + robot_cloud.colwise().maxCoeff()) * 0.5;
    // Eigen::RowVector3d centroid_object = object_cloud.colwise().mean();
    Eigen::RowVector3d centroid_object = (object_cloud.colwise().minCoeff() + object_cloud.colwise().maxCoeff()) * 0.5;

    // Center the point clouds
    Eigen::MatrixXd centered_robot = robot_cloud.rowwise() - centroid_robot;
    Eigen::MatrixXd centered_object = object_cloud.rowwise() - centroid_object;

    // Compute PCA
    Eigen::Matrix3d robot_pca = getMajorAxis(centered_robot);
    Eigen::Matrix3d object_pca = getMajorAxis(centered_object);

    // Extract principal axes
    Eigen::Vector3d robot_axis1 = robot_pca.col(0);
    Eigen::Vector3d robot_axis2 = robot_pca.col(1);
    Eigen::Vector3d robot_axis3 = robot_pca.col(2);
    Eigen::Vector3d object_axis1 = object_pca.col(0);
    Eigen::Vector3d object_axis2 = object_pca.col(1);
    Eigen::Vector3d object_axis3 = object_pca.col(2);

	double max_proj;
	Eigen::Vector3d offset;
	Eigen::Matrix3d target_rot;
	Eigen::Matrix3d R;
    Eigen::Vector3d centroid_robot_v;
    Eigen::Vector3d centroid_object_v;
    Eigen::Vector3d t;
	

	if (base_ini.empty()) {	
		if (_robot_name == "barrett") {
			double max_proj = (centered_object * robot_axis3).maxCoeff();
			offset << 0, 0, -max_proj;

			// Build target rotation matrix
			target_rot.col(0) = object_axis2;
			target_rot.col(1) = -object_axis3;
			target_rot.col(2) = -object_axis1;
			R = target_rot * robot_pca.transpose();

			// Compute translation
			centroid_robot_v = centroid_robot.transpose();
			centroid_object_v = centroid_object.transpose();
			t = centroid_object_v - R * centroid_robot_v + object_pca * offset;
		}
		else if (_robot_name == "allegro") {
			// Additional translation offset 
			double max_proj = (centered_object * robot_axis3).maxCoeff();
			// offset << 0, 0, -max_proj;
			offset << 0, 0, 0;

			// Build target rotation matrix
			target_rot.col(0) = object_axis3;
			target_rot.col(1) = object_axis1;
			target_rot.col(2) = object_axis2;

			R = target_rot;

			// Compute translation
			centroid_robot_v = centroid_robot.transpose();
			centroid_object_v = centroid_object.transpose();
			t = centroid_object_v - R * centroid_robot_v + object_pca * offset;
		}
		else if (_robot_name == "leaphand") {
			// Additional translation offset 
			double max_proj = (centered_object * robot_axis2).maxCoeff();
			// offset << 0, 0, -max_proj;
			// offset << 0, -max_proj, 0;
			offset << max_proj, -max_proj, 0;
			// offset << 0, 0, 0;

			// Build target rotation matrix
			target_rot.col(0) = object_axis2;
			target_rot.col(1) = object_axis1;
			target_rot.col(2) = object_axis3;

			R = target_rot;

			// Compute translation
			centroid_robot_v = centroid_robot.transpose();
			centroid_object_v = centroid_object.transpose();
			t = centroid_object_v - R * centroid_robot_v + object_pca * offset;
		}
		else if (_robot_name == "shadowhand") {		
			// Additional translation offset 
			double max_proj = (centered_object * robot_axis2).maxCoeff();
			offset << 0, -max_proj, 0;

			// Build target rotation matrix
			target_rot.col(0) = -object_axis3;
			target_rot.col(1) = object_axis1;
			target_rot.col(2) = -object_axis2;
			R = target_rot * robot_pca.transpose();

			// Compute translation
			centroid_robot_v = centroid_robot.transpose();
			centroid_object_v = centroid_object.transpose();
			t = centroid_object_v - R * centroid_robot_v + object_pca * offset;
		}
		else {
			std::cerr << "AdjustBase: Only support shadowhand, "
						<< "barrett, allegro, and leaphand." << std::endl;
			exit(0);
		}

		// Build 4x4 transformation matrix
		Eigen::Matrix4d T_ro = Eigen::Matrix4d::Identity();
		T_ro.block<3, 3>(0, 0) = R;
		T_ro.block<3, 1>(0, 3) = t;

		new_base = T_ro * grasp::Vec2d2Eigen(robot_solver_ptr -> _base);
	}
	else {
		new_base = grasp::Vec2d2Eigen(base_ini);
	}

	Eigen::MatrixXd new_base_euler(6, 1);
	new_base_euler << std::atan2(new_base(2,1), new_base(2,2)),
						std::atan2(new_base(1,0), new_base(0,0)),
						std::asin(-new_base(2,0)),
						new_base(0,3), new_base(1,3), new_base(2,3);
	
	// reset limits of roll, yaw, pitch
	if (base_ini.empty()) {
		if (_robot_name == "allegro") {
			_robot_solver->ROLL_MAX = new_base_euler(0) + M_PI/12;
			_robot_solver->ROLL_MIN = new_base_euler(0) - M_PI/12;
			_robot_solver->YAW_MAX = new_base_euler(1) + M_PI/12;
			_robot_solver->YAW_MIN = new_base_euler(1) - M_PI/12;
			_robot_solver->PITCH_MAX = new_base_euler(2) + M_PI/12;
			_robot_solver->PITCH_MIN = new_base_euler(2) - M_PI/12;
		}
		if (_robot_name == "leaphand") {
			_robot_solver->ROLL_MAX = new_base_euler(0) + M_PI/2;
			_robot_solver->ROLL_MIN = new_base_euler(0) - M_PI/2;
			_robot_solver->YAW_MAX = new_base_euler(1) + M_PI/2;
			_robot_solver->YAW_MIN = new_base_euler(1) - M_PI/2;
			_robot_solver->PITCH_MAX = new_base_euler(2) + M_PI/2;
			_robot_solver->PITCH_MIN = new_base_euler(2) - M_PI/2;
		}
	} 
	else {
		// tighter limits when initial base is provided
		_robot_solver->ROLL_MAX = new_base_euler(0) + M_PI/24;
		_robot_solver->ROLL_MIN = new_base_euler(0) - M_PI/24;
		_robot_solver->YAW_MAX = new_base_euler(1) + M_PI/24;
		_robot_solver->YAW_MIN = new_base_euler(1) - M_PI/24;
		_robot_solver->PITCH_MAX = new_base_euler(2) + M_PI/24;
		_robot_solver->PITCH_MIN = new_base_euler(2) - M_PI/24;
		// fix the base translation
		_robot_solver->fix_base_translation = true;
	}
	
	UpdateBaseEuler6D(grasp::Eigen2Vec(new_base_euler));
	robot_solver_ptr -> FKRollout();
	robot_solver_ptr -> ComputeLgrgn();

} ;

void GraspOpt::ComputeGrasp(std::vector<double>& theta_solution,
							const std::vector<std::vector<double>>& object_pts,
							const std::vector<std::vector<double>>& base_ini) {
	
	_robot_solver -> SetCdistComputeParams(solid_joint_id, solid_joint_rad,
											solid_link_id, solid_link_rad, 
											object_pts, 
											must_touch_obj_ee_id, must_touch_obj_ee_rad);
	// _obj_solver -> _object_position_origin = _robot_solver -> _object_position;
	_obj_solver -> InitParams(must_touch_obj_ee_id, object_pts);
	
	if(_verbose) std::cout << "[ComputeGrasp] Initialize" << std::endl;
	
	AdjustBase(_robot_solver, _obj_solver, _theta_ini, true, _verbose, base_ini);
	adjusted_initial_active_ee = grasp::Eigen2Vec2d(_robot_solver->_active_ee_position);
	adjusted_initial_pts_with_parent.resize(_robot_solver->_link_poses.size());
	for (int i = 0; i < adjusted_initial_pts_with_parent.size(); ++i) {
		for (int j = 0; j < 3; ++j) 
			adjusted_initial_pts_with_parent[i].emplace_back(
				_robot_solver->_link_poses[i][j][3]
			);
		for (int j = 0; j < 3; ++j) {
			if (_robot_solver->_parent[i] == -1) 
				adjusted_initial_pts_with_parent[i].emplace_back(
					_robot_solver->_base[j][3]
				);
			else
				adjusted_initial_pts_with_parent[i].emplace_back(
					_robot_solver->_link_poses[_robot_solver->_parent[i]][j][3]
				);
		}
	}

	// adjusted_initial_active_ee = yu::Eigen2Vec2d(_robot_solver->_must_touch_ee_position);

	if(_verbose) std::cout << "[ComputeGrasp] Adjust Base" << std::endl;
	
	int game_converge = GraspSolver(_robot_solver, _obj_solver, _theta_ini, true, _verbose);
	
	if (_verbose) std::cout << "[ComputeGrasp] Game converge: " << game_converge << std::endl;

	if (game_converge == 0) {
		delete _robot_solver;
    	_robot_solver = new RobotConfigOptimizer();
		_robot_solver->SetInitPoses(_m0s) ;
		_robot_solver->InitKinematicParams(joint_limits, _a, _d, _alpha, _parent) ;
		_robot_solver -> SetCdistComputeParams(solid_joint_id, solid_joint_rad,
												solid_link_id, solid_link_rad, 
												object_pts, 
												must_touch_obj_ee_id, must_touch_obj_ee_rad);
		_obj_solver -> InitParams(solid_joint_id, object_pts);
		AdjustBase(_robot_solver, _obj_solver, _theta_ini, true, _verbose, base_ini);
		adjusted_initial_active_ee = grasp::Eigen2Vec2d(_robot_solver->_active_ee_position);
		adjusted_initial_pts_with_parent.resize(_robot_solver->_link_poses.size());
		for (int i = 0; i < adjusted_initial_pts_with_parent.size(); ++i) {
			for (int j = 0; j < 3; ++j) 
				adjusted_initial_pts_with_parent[i].emplace_back(
					_robot_solver->_link_poses[i][j][3]
				);
			for (int j = 0; j < 3; ++j) {
				if (_robot_solver->_parent[i] == -1) 
					adjusted_initial_pts_with_parent[i].emplace_back(
						_robot_solver->_base[j][3]
					);
				else
					adjusted_initial_pts_with_parent[i].emplace_back(
						_robot_solver->_link_poses[_robot_solver->_parent[i]][j][3]
					);
			}
		}

		// directly optimize
		GraspSolver(_robot_solver, _theta_ini, true, _verbose);
	}

	optimized_active_ee = grasp::Eigen2Vec2d(_robot_solver->_active_ee_position);
	optimized_pts_with_parent.resize(_robot_solver->_link_poses.size());
	for (int i = 0; i < optimized_pts_with_parent.size(); ++i) {
		for (int j = 0; j < 3; ++j) 
			optimized_pts_with_parent[i].emplace_back(
				_robot_solver->_link_poses[i][j][3]
			);
		for (int j = 0; j < 3; ++j) {
			if (_robot_solver->_parent[i] == -1) 
				optimized_pts_with_parent[i].emplace_back(
					_robot_solver->_base[j][3]
				);
			else
				optimized_pts_with_parent[i].emplace_back(
					_robot_solver->_link_poses[_robot_solver->_parent[i]][j][3]
				);
		}
	}

	theta_solution = _robot_solver->joint_solution;

	base_trans = {_robot_solver -> _base[0][3], 
				  _robot_solver -> _base[1][3],
				  _robot_solver -> _base[2][3]};
	base_euler_angle = {_robot_solver -> _euler[0], 
				  _robot_solver -> _euler[1],
				  _robot_solver -> _euler[2]};
	base_quat = _robot_solver -> GetBaseQuaternion();
	
	ee_penalty = _robot_solver->_ee_penalty + _robot_solver->_max_cdist_penalty;
	colli_penalty = _robot_solver->_colli_penalty + _robot_solver->_max_self_colli_penalty;
	penalty = _robot_solver->_ee_penalty + _robot_solver->_colli_penalty;

} ;

} // end namespace grasp
