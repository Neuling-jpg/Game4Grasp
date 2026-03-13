
#pragma once
#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>
#include "kinematics_models/kinematics_model.hpp"
#include "kinematics_models/robot_model_sheets/allegro.hpp"
#include "kinematics_models/robot_model_sheets/barrett.hpp"
#include "kinematics_models/robot_model_sheets/leaphand.hpp"
#include "kinematics_models/robot_model_sheets/shadowhand.hpp"

namespace grasp {

class RobotRegistry {
public:
  using Factory = std::function<KinematicModel()>;

  static RobotRegistry& instance() {
    static RobotRegistry R;
    return R;
  }

  void reg(std::string name, Factory f) {
    factories_.emplace(std::move(name), std::move(f));
  }

  bool has(std::string_view name) const {
    return factories_.find(std::string(name)) != factories_.end();
  }

  KinematicModel make(std::string_view name) const {
    auto it = factories_.find(std::string(name));
    if (it == factories_.end()) throw std::runtime_error("Unknown robot: " + std::string(name));
    return (it->second)();
  }

private:
  RobotRegistry() {
    factories_["barrett"] = []{
      return build_model_from_specs(robots::kBarrett.data(), robots::kBarrett.size());
    };
    factories_["allegro"] = []{
      return build_model_from_specs(robots::kAllegro.data(), robots::kAllegro.size());
    };
    factories_["leaphand"] = []{
      return build_model_from_specs(robots::kLeapHand.data(), robots::kLeapHand.size());
    };
    factories_["shadowhand"] = []{
      return build_model_from_specs(robots::kShadowHand.data(), robots::kShadowHand.size());
    };
  }
  std::unordered_map<std::string, Factory> factories_;
};

} // namespace game4grasp