#ifndef ROBOT_MODEL_H_
#define ROBOT_MODEL_H_

#include <string_view>
#include "core/grasp_solver.hpp"
#include "kinematics_models/kinematics_model.hpp"

namespace grasp{

/**
 * @brief An interface class for robot arms.
 */
class RobotModel{
public:
	/**
	 * @brief
	 */
	RobotModel() {};
	/**
	 * @brief
	 */
	virtual ~RobotModel() {};
	/**
	 * @brief
	 */
	virtual std::vector<std::string> GetLinkNames() = 0;
	/**
	 * @brief
	 */
	virtual void UpdateBaseFrame(const std::vector<std::vector<double>>& base) = 0;
	/**
	 * @brief
	 */
	virtual void ComputeFK(const std::vector<double>& theta) = 0;
	/**
	 * @brief
	 */
	virtual std::vector<std::vector<double> > GetLinkPose(int k) = 0;
	/**
	 * @brief
	 */
	int num_joints = -1;
	/**
	 * @brief
	 */
	std::vector<std::vector<double> > joint_limits;
};

/**
 * @brief 
 */
class GraspOpt : public RobotModel
{
public:
	/**
	 * @brief
	 */
	GraspOpt (const std::string_view& robot_name, const bool& verbose = false) ;
	/**
	 * @brief
	 */
	virtual ~GraspOpt () ;
	/**
	 * @brief Get link names.
	 */
	virtual std::vector<std::string> GetLinkNames() override ;
	/**
	 * @brief
	 */
	virtual void UpdateBaseFrame(const std::vector<std::vector<double>>& base) override ;
	/**
	 * @brief
	 */
	virtual void UpdateBaseEuler6D(const std::vector<double>& euler) ;  // input: euler angle + position
	/**
	 * @brief
	 */
	void GetBaseFrame(std::vector<std::vector<double>>& base) ;
	std::vector<std::vector<double>> GetBaseFrame() ;

	/**
	 * @brief
	 */
	virtual void SetCdistParams(const std::vector<int>& solid_joint_id,
								const std::vector<double>& solid_joint_rad,
								const std::vector<std::vector<int>>& solid_link_id,
								const std::vector<double>& solid_link_rad,
								const std::vector< std::vector<double> >& object_points,
								const std::vector< std::vector<double> >& cdist_target) ;
	/**
	 * @brief
	 */
	virtual void ComputeFK(const std::vector<double>& theta) override ;
	/**
	 * @brief
	 */
	virtual std::vector<std::vector<double> > GetLinkPose(int k) override ;
	/**
	 * @brief
	 */
	virtual void ComputeIK(const std::vector<double>& theta_ini, 
							std::vector<double>& theta_solution,
							const std::map<int, std::vector<double>>& ee_target_position,
							const std::vector<std::vector<double>>& points) ;
	virtual void ComputeIKCdist(std::vector<double>& theta_solution,
								const std::vector<double>& theta_ini, 
								const std::vector<std::vector<int>>& solid_link_id,
								const std::vector<double>& solid_link_rad,
								const std::vector<std::vector<double>>& object_pts,
								const std::vector<std::vector<double>>& cdist_target,
								const std::vector<std::vector<double>>& obstacle_pts);
	/**
	 * @brief 
	 */
	Eigen::Matrix3d getMajorAxis(const Eigen::MatrixXd& pointcloud) ;
	/**
	 * @brief 
	 */
	void AdjustBase(RobotConfigOptimizer* robot_solver_ptr, 
					ObjectPoseOptimizer* object_solver_ptr ,
					const std::vector<double>& theta, 
					const bool& use_cdist, const bool& _verbose,
					const std::vector<std::vector<double>>& base_ini) ;
	std::vector<std::vector<double>> adjusted_initial_active_ee;  // adjusted active end-effector position, used in the base adjustment
	std::vector<std::vector<double>> adjusted_initial_pts_with_parent;  // adjusted active end-effector position, used in the base adjustment
	/**
	 * @brief 
	 */
	virtual void ComputeGrasp(std::vector<double>& theta_solution,
								// const std::vector<double>& theta_ini, 
								const std::vector<std::vector<double>>& object_pts,
								const std::vector<std::vector<double>>& base_ini) ;
	double ee_penalty = 0, colli_penalty = 0, penalty = 0;  // penalty for the IK solution, used in the optimization
	std::vector<int> active_joint_id;
	std::vector<int> solid_joint_id;
	std::vector<std::vector<int>> solid_link_id;
	std::vector<int> must_touch_obj_ee_id;
	std::vector<double> solid_joint_rad;
	std::vector<double> solid_link_rad;
	std::vector<double> must_touch_obj_ee_rad;
	std::vector<double> base_trans, base_euler_angle, base_quat;
	std::vector<std::vector<double>> optimized_active_ee;
	std::vector<std::vector<double>> optimized_pts_with_parent;
	std::vector<double> _theta_ini;
protected:
	/**
	 * @brief
	 */
	virtual void _init(const KinematicModel& km) ;
	bool _verbose;

	std::vector< std::vector<std::vector<double> > > _m0s;  // initial pose of each link, n x 4 x 4
	
	// std::map<int, int> _parent;  // parent link index, e.g., parent[0] = -1, parent[1] = 0, parent[2] = 1, etc.
	std::vector<int> _parent;  // parent link index, e.g., parent[0] = -1, parent[1] = 0, parent[2] = 1, etc.
	std::vector<std::vector<double> > _jl;  // joint limits, n x 2
	std::vector<double> _a;  // DH parameters 'a': _a[i] indicates length of link i-1, n x 1
	std::vector<double> _d;  // DH parameters 'd': _d[i] indicates offset along previous z to the common normal, n x 1
	std::vector<double> _alpha;  // DH parameters 'alpha': _alpha[i] indicates twist angle of link i-1, n x 1
	std::vector<std::string> _link_names;

	ForwardKinematics* _fk_ptr;
	RobotConfigOptimizer* _robot_solver;
	ObjectPoseOptimizer* _obj_solver;
	std::string_view _robot_name;
	std::vector<int> _colli_object_idx;

	std::vector< std::vector<double> > _colli_object_radius;  // r_ellipsoid and r_sphere

};

} // end namespace grasp

#endif  // ROBOT_MODEL_H_
