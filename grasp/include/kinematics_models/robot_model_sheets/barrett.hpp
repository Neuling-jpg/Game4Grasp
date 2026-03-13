#pragma once
#include <array>
#include "kinematics_models/kinematics_model.hpp"

namespace grasp::robots {
inline const std::array<JointSpec, 22> kBarrett = {
  /* 
  a, d, alpha: DH params (we use modified DH)
  lim_min, lim_max: joint angle limits
  ini_angle: initial joint angle
  parent: parent joint idx
  jrad: joint collision sphere radius, =0 means no collision sphere
  lrad: link collision ellipsoid radius, =0 means no collision ellipsoid
  trad: touch object sphere radius, =-1 means this point doesnt need to touch the object
  name: joint name
  */

  // a     , d      , alpha    , lim_min, lim_max, ini_angle, parent, jrad,  lrad,  trad, name
  0        , 0.059  , 0        , 0      , 0      , 0         , -1   , 0.002, 0    , -1   , "J00A" ,  // 0
  0        , 0      , 0        , M_PI/2 , M_PI/2 , M_PI/2    , 0    , 0    , 0    , -1   , "J00B" ,  // 1

  0.025    , 0      , M_PI     , -M_PI/2, -M_PI/2, -M_PI/2   , 1    , 0.002, 0.010, 0.023, "J01" ,  // 2
  0        , 0      , 0        , 0      , M_PI   , M_PI/9    , 2    , 0.020, 0.010, -1   , "J11" ,  // 3
  0.050    , 0      , -M_PI/2  , 0      ,7*M_PI/9, M_PI/4    , 3    , 0.020, 0.020, -1   , "J12" ,  // 4
  0.070    , 0      , 0        , 0      , M_PI/4 , M_PI/8    , 4    , 0.020, 0.020, -1   , "J13" ,  // 5
  0.049    , 0      , 0        , 0      , 0      , 0         , 5    , 0.020, 0.020, 0.023, "J1t" ,  // 6

  -0.025   , 0      , 0        , M_PI/2 , M_PI/2 , M_PI/2    , 1    , 0.002, 0.010, 0.023, "J02" ,  // 7
  0        , 0      , 0        , 0      , M_PI   , M_PI/9    , 7    , 0.020, 0.010, -1   , "J21" ,  // 8
  0.050    , 0      , M_PI/2   , 0      ,7*M_PI/9, M_PI/4    , 8    , 0.020, 0.020, -1   , "J22" ,  // 9
  0.070    , 0      , 0        , 0      , M_PI/4 , M_PI/8    , 9    , 0.020, 0.020, -1   , "J23" ,  // 10
  0.049    , 0      , 0        , 0      , 0      , 0         , 10   , 0.020, 0.020, 0.023, "J2t" ,  // 11

  0.050    , 0      , 0        , 0      , 0      , 0         , 0    , 0.002, 0.010, 0.023, "J31" ,  // 12
  0        , 0      , M_PI/2   , 0      ,7*M_PI/9, M_PI/4    , 12   , 0.020, 0.010, -1   , "J32" ,  // 13
  0.070    , 0      , 0        , 0      , M_PI/4 , M_PI/8    , 13   , 0.020, 0.020, -1   , "J33" ,  // 14
  0.049    , 0      , 0        , 0      , 0      , 0         , 14   , 0.020, 0.020, 0.023, "J3t" ,  // 15
};

} // namespace grasp::robots