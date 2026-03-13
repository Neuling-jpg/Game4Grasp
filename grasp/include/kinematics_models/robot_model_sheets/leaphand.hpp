#pragma once
#include <array>
#include "kinematics_models/kinematics_model.hpp"

namespace grasp::robots {
inline double AAA = 0.019;
inline double BBB = AAA + 0.003;
inline double CCC = 0.015;
inline double DDD = CCC + 0.003;
inline const std::array<JointSpec, 50> kLeapHand = {
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
  0        , 0      , 0        , 0      , 0      , 0         , -1   , 0    , 0    , -1   , "base0" ,  // 0
  0.019    , 0.091  , 0        , 0      , 0      , 0         , 0    , CCC  , 0    , -1   , "base1" ,  // 1

  0	       , 0.061	, -M_PI/2  , -M_PI/2, -M_PI/2, -M_PI/2   , 1    , CCC  , CCC  , DDD  , "j1-Bridge" ,  // 2
  0        , 0      , 0        , 0      , 2.544  , 0.714     , 2    , 0    , 0    , -1   , "j1A"  ,  // 3
  0        , 0      , 0        , -0.314 , -0.314 , -0.314    , 3    , 0    , 0    , -1   , "j1B"  ,  // 4
  0.038    , 0      , -M_PI/2  , 0      , 2.094  , 0.547     , 4    , AAA  , AAA  , -1   , "j0A"  ,  // 5
  0        , 0      , 0        , -1.047 , -1.047 , -1.047    , 5    , 0    , 0    , -1   , "j0B"  ,  // 6
  0.025    , 0      , M_PI/2   , 0      , 2.391  , 0.906     , 6    , AAA  , AAA  , BBB  , "j2A"  ,  // 7
  0        , 0      , 0        , -0.506 , -0.506 , -0.506    , 7    , 0    , 0    , -1   , "j2B"  ,  // 8
  0.036    , 0      , 0        , 0      , 2.408  , 0.766     , 8    , AAA  , AAA  , BBB  , "j3A"  ,  // 9
  0        , 0      , 0        , -0.366 , -0.366 , -0.366    , 9    , 0    , 0    , -1   , "j3B"  ,  // 10
  // 0.048    , 0      , 0        , 0      , 0      , 0         , 10   , AAA  , AAA  , BBB  , "index_tip"  ,  // 11
  0.038    , 0      , 0        , 0      , 0      , 0         , 10   , AAA  , AAA  , BBB  , "index_tip"  ,  // 11
  
  0	       , 0.016	, -M_PI/2  , -M_PI/2, -M_PI/2, -M_PI/2   , 1    , CCC  , CCC  , DDD  , "j5-Bridge" ,  // 12
  0        , 0      , 0        , 0      , 2.544  , 0.714     , 12   , 0    , 0    , -1   , "j5A"  ,  // 13
  0        , 0      , 0        , -0.314 , -0.314 , -0.314    , 13   , 0    , 0    , -1   , "j5B"  ,  // 14
  0.038    , 0      , -M_PI/2  , 0      , 2.094  , 1.047     , 14   , AAA  , AAA  , -1   , "j4A"  ,  // 15
  0        , 0      , 0        , -1.047 , -1.047 , -1.047    , 15   , 0    , 0    , -1   , "j4B"  ,  // 16
  0.025    , 0      , M_PI/2   , 0      , 2.391  , 0.906     , 16   , AAA  , AAA  , BBB  , "j6A"  ,  // 17
  0        , 0      , 0        , -0.506 , -0.506 , -0.506    , 17   , 0    , 0    , -1   , "j6B"  ,  // 18
  0.036    , 0      , 0        , 0      , 2.408  , 0.766     , 18   , AAA  , AAA  , BBB  , "j7A"  ,  // 19
  0        , 0      , 0        , -0.366 , -0.366 , -0.366    , 19   , 0    , 0    , -1   , "j7B"  ,  // 20
  // 0.048    , 0      , 0        , 0      , 0      , 0         , 20   , AAA  , AAA  , BBB  , "middle_tip"  ,  // 21
  0.038    , 0      , 0        , 0      , 0      , 0         , 20   , AAA  , AAA  , BBB  , "middle_tip"  ,  // 21
  
  0	       , -0.030 , -M_PI/2  , -M_PI/2, -M_PI/2, -M_PI/2   , 1    , CCC  , CCC  , DDD  , "j9-Bridge" ,  // 22
  0        , 0      , 0        , 0      , 2.544  , 0.714     , 22   , 0    , 0    , -1   , "j9A"  ,  // 23
  0        , 0      , 0        , -0.314 , -0.314 , -0.314    , 23   , 0    , 0    , -1   , "j9B"  ,  // 24
  0.038    , 0      , -M_PI/2  , 0      , 2.094  , 1.547     , 24   , AAA  , AAA  , -1   , "j8A"  ,  // 25
  0        , 0      , 0        , -1.047 , -1.047 , -1.047    , 25   , 0    , 0    , -1   , "j8B"  ,  // 26
  0.025    , 0      , M_PI/2   , 0      , 2.391  , 0.906     , 26   , AAA  , AAA  , BBB  , "j10A"  ,  // 27
  0        , 0      , 0        , -0.506 , -0.506 , -0.506    , 27   , 0    , 0    , -1   , "j10B"  ,  // 28
  0.036    , 0      , 0        , 0      , 2.408  , 0.766     , 28   , AAA  , AAA  , BBB  , "j11A"  ,  // 29
  0        , 0      , 0        , -0.366 , -0.366 , -0.366    , 29   , 0    , 0    , -1   , "j11B"  ,  // 30
  // 0.048    , 0      , 0        , 0      , 0      , 0         , 30   , AAA  , AAA  , BBB  , "ring_tip"  ,  // 31
  0.038    , 0      , 0        , 0      , 0      , 0         , 30   , AAA  , AAA  , BBB  , "ring_tip"  ,  // 31

  0.022    , 0.016	, 0        , M_PI/2 , M_PI/2 , M_PI/2    , 0    , AAA  , 0    , -1   , "j12-Bridge1" ,  // 32
  0.037    , 0    	, M_PI     , -M_PI/2, -M_PI/2, -M_PI/2   , 32   , AAA  , AAA  , -1   , "j12-Bridge2" ,  // 33
  0        , 0      , 0        , 0      , 2.398  , 1.849     , 33   , 0    , 0    , -1   , "j12A"  ,  // 34
  0        , 0      , 0        , -0.349 , -0.349 , -0.349    , 34   , 0    , 0    , -1   , "j12B"  ,  // 35
  0        , 0      , -M_PI/2  , 0      , 2.913  , 1.47      , 35   , 0    , 0    , -1   , "j13A"  ,  // 36B  
  0        , 0      , 0        , -0.47  , -0.47  , -0.47     , 36   , 0    , 0    , -1   , "j13B"  ,  // 37
  0        , 0      , M_PI/2   , M_PI/2 , M_PI/2 , M_PI/2    , 37   , 0    , 0    , -1   , "j14-Bridge"  ,  // 38
  0.017    , 0      , -M_PI/2  , 0      , 3.1    , 1.5       , 38   , AAA  , AAA  , -1   , "j14A"  ,  // 39
  0        , 0      , 0        , -1.2   , -1.2   , -1.2      , 39   , 0    , 0    , -1   , "j14B"  ,  // 40
  0.047    , 0      , 0        , 0      , 3.22   , 1.64      , 40   , AAA  , AAA  , -1   , "j15A"  ,  // 41
  0        , 0      , 0        , -1.34  , -1.34  , -1.34     , 41   , 0    , 0    , -1   , "j15B"  ,  // 42
  // 0.060    , 0      , 0        , 0      , 0      , 0         , 42   , AAA  , AAA  , BBB  , "thumb_tip"  ,  // 43
  0.050    , 0      , 0        , 0      , 0      , 0         , 42   , AAA  , AAA  , BBB  , "thumb_tip"  ,  // 43

  // additional joints
  0.019    , 0.070  , 0        , 0      , 0      , 0         , 0    , CCC  , CCC  , -1   , "palm1" ,  // 44
  0	       , 0.061	, -M_PI/2  , -M_PI/2, -M_PI/2, -M_PI/2   , 44   , CCC  , CCC  , -1   , "palm2" ,  // 45
  0	       , -0.030 , -M_PI/2  , -M_PI/2, -M_PI/2, -M_PI/2   , 44   , CCC  , CCC  , -1   , "palm3" ,  // 46
  0.019    , 0.040  , 0        , 0      , 0      , 0         , 0    , CCC  , CCC  , -1   , "palm4" ,  // 47
  0	       , 0.061	, -M_PI/2  , -M_PI/2, -M_PI/2, -M_PI/2   , 47   , CCC  , CCC  , -1   , "palm5" ,  // 48
  0	       , -0.030 , -M_PI/2  , -M_PI/2, -M_PI/2, -M_PI/2   , 47   , CCC  , CCC  , -1   , "palm6" ,  // 49
};

} // namespace grasp::robots