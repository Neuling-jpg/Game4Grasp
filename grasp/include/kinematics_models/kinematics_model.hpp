#pragma once
#include <vector>
#include <array>
#include <string>
#include <string_view>
#include <memory>
#include <cassert>

namespace grasp {

struct JointSpec {
  double a;
  double d;
  double alpha;
  double lim_min;
  double lim_max;
  double ini_angle;
  int    parent;
  double jrad;
  double lrad;
  double trad;
  std::string_view name;
};

struct KinematicModel {
  int num_joints{0};
  std::vector<double> a, d, alpha;
  std::vector<int>    parent;
  std::vector<std::array<double,2>> limits;
  std::vector<double> ini_angles;
  std::vector<std::vector<std::vector<double>>> m0s;
  std::vector<double> jrad;
  std::vector<double> lrad;
  std::vector<double> trad;
  std::vector<std::string> name;

  bool valid() const {
    if (num_joints <= 0) return false;
    if ((int)a.size()!=num_joints || (int)d.size()!=num_joints ||
        (int)alpha.size()!=num_joints || (int)parent.size()!=num_joints ||
        (int)limits.size()!=num_joints || (int)ini_angles.size()!=num_joints || 
        (int)name.size()!=num_joints || (int)jrad.size()!=num_joints || 
        (int)lrad.size()!=num_joints || (int)trad.size()!=num_joints) return false;
    for (int i=0;i<num_joints;++i) {
      if (parent[i] >= num_joints) return false;
    }
    return true;
  }
};

// build kinematics model from the specs chart
inline KinematicModel build_model_from_specs(const JointSpec* specs, std::size_t n) {
  KinematicModel km;
  km.num_joints = static_cast<int>(n);
  km.a.resize(n); km.d.resize(n); km.alpha.resize(n);
  km.parent.resize(n); km.limits.resize(n); 
  km.ini_angles.resize(n); km.m0s.resize(n); km.name.resize(n);
  km.jrad.resize(n); km.lrad.resize(n); km.trad.resize(n);
  for (std::size_t i=0;i<n;++i) {
    km.a[i] = specs[i].a;
    km.d[i] = specs[i].d;
    km.alpha[i] = specs[i].alpha;
    km.parent[i] = specs[i].parent;
    km.limits[i] = { specs[i].lim_min, specs[i].lim_max };
    km.ini_angles[i] = specs[i].ini_angle;
    km.name[i] = specs[i].name;
    km.jrad[i] = specs[i].jrad;
    km.lrad[i] = specs[i].lrad;
    km.trad[i] = specs[i].trad;
  }
  assert(km.valid());
  return km;
}

} // namespace grasp