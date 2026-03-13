#pragma once
#include <array>
#include "kinematics_models/kinematics_model.hpp"

namespace grasp::robots {
inline double sAA = 0.013;
inline double sBB = 0.011;
inline double sCC = 0.009;
inline double sDD = 0.012;
inline const std::array<JointSpec, 41> kShadowHand = { 
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
  0.21301, -0.01  ,  M_PI    , -0.489 ,  0.140 , 0         , -1,     sAA  , sAA  , -1   , "WR2" ,  // 0
  0.034  ,  0.0   , -M_PI/2  , -0.698 ,  0.489 , 0         , 0 ,     sAA  , sAA  , -1   , "WR1" ,  // 1

  0.095  ,  0.034 ,  0.0     ,  0.0   ,  0.0   , 0         , 1 ,     0    , sBB  , -1   , "FF-Bridge4" ,  // 2
  0.0    ,  0.0   , -M_PI/2  , -0.349 ,  0.349 , 0         , 2 ,     sBB  , 0    , -1   , "FF4" ,  // 3
  0.0    ,  0.0   ,  M_PI/2  , -0.262 ,  1.571 , M_PI/12   , 3 ,     0    , 0    , -1   , "FF3" ,  // 4
  0.045  ,  0.0   ,  0.0     ,  0.0   ,  1.571 , M_PI/12   , 4 ,     sCC  , sBB  , -1   , "FF2" ,  // 5
  0.025  ,  0.0   ,  0.0     ,  0.0   ,  1.571 , M_PI/12   , 5 ,     sCC  , sCC  , -1   , "FF1" ,  // 6
  0.026  ,  0.0   ,  0.0     ,  0.0   ,  0.0   , 0         , 6 ,     sCC  , sCC  , sDD  , "FF-Tip" ,  // 7

  0.099  ,  0.012 ,  0.0     ,  0.0   ,  0.0   , 0         , 1 ,     0    , sBB  , -1   , "MF-Bridge4" ,  // 8
  0.0    ,  0.0   , -M_PI/2  , -0.349 ,  0.349 , 0         , 8 ,     sBB  , 0    , -1   , "MF4" ,  // 9
  0.0    ,  0.0   ,  M_PI/2  , -0.262 ,  1.571 , M_PI/12   , 9 ,     0    , 0    , -1   , "MF3" ,  // 10
  0.045  ,  0.0   ,  0.0     ,  0.0   ,  1.571 , M_PI/12   , 10,     sCC  , sBB  , -1   , "MF2" ,  // 11
  0.025  ,  0.0   ,  0.0     ,  0.0   ,  1.571 , M_PI/12   , 11,     sCC  , sCC  , -1   , "MF1" ,  // 12
  0.026  ,  0.0   ,  0.0     ,  0.0   ,  0.0   , 0         , 12,     sCC  , sCC  , sDD  , "MF-Tip" ,  // 13

  0.095  , -0.01  ,  0.0     ,  0.0   ,  0.0   , 0         , 1 ,     0    , sBB  , -1   , "RF-Bridge4" ,  // 14
  0.0    ,  0.0   ,  M_PI/2  , -0.349 ,  0.349 , 0         , 14,     sBB  , 0    , -1   , "RF4" ,  // 15
  0.0    ,  0.0   , -M_PI/2  , -0.262 ,  1.571 , M_PI/12   , 15,     0    , 0    , -1   , "RF3" ,  // 16
  0.045  ,  0.0   ,  0.0     ,  0.0   ,  1.571 , M_PI/12   , 16,     sCC  , sBB  , -1   , "RF2" ,  // 17
  0.025  ,  0.0   ,  0.0     ,  0.0   ,  1.571 , M_PI/12   , 17,     sCC  , sCC  , -1   , "RF1" ,  // 18
  0.026  ,  0.0   ,  0.0     ,  0.0   ,  0.0   , 0         , 18,     sCC  , sCC  , sDD  , "RF-Tip" ,  // 19

  0.0    , -0.047 ,  0.0     ,  M_PI/2,  M_PI/2, M_PI/2    , 1 ,     0    , 0    , -1   , "LF-Bridge5" ,  // 20
  0.0    ,  0.030 ,  0.9599  ,  0.0   ,  0.785 , 0.05      , 20,     sBB  , 0    , -1   , "LF5" ,  // 21
  0.0    ,  0.0   , -0.9599  , -M_PI/2, -M_PI/2, -M_PI/2   , 21,     0    , 0    , -1   , "LF-Bridge4-1" ,  // 22
  0.066  ,  0.0   ,  0.0     ,  0.0   ,  0.0   , 0         , 22,     0    , sBB  , -1   , "LF-Bridge4-2" ,  // 23
  0.0    ,  0.0   ,  M_PI/2  , -0.349 ,  0.349 , -0.2      , 23,     sCC  , 0    , -1   , "LF4" ,  // 24
  0.0    ,  0.0   , -M_PI/2  , -0.262 ,  1.571 , M_PI/12   , 24,     0    , 0    , -1   , "LF3" ,  // 25
  0.045  ,  0.0   ,  0.0     ,  0.0   ,  1.571 , M_PI/12   , 25,     sCC  , sCC  , -1   , "LF2" ,  // 26
  0.025  ,  0.0   ,  0.0     ,  0.0   ,  1.571 , M_PI/12   , 26,     sCC  , sCC  , -1   , "LF1" ,  // 27
  0.026  ,  0.0   ,  0.0     ,  0.0   ,  0.0   , 0         , 27,     sCC  , sCC  , sDD  , "LF-Tip" ,  // 28

  0.0    ,  0.005 ,  0.0     ,  M_PI/2,  M_PI/2, M_PI/2    , 1 ,     0    , 0    , -1   , "TH-Bridge5" ,  // 29
  0.0085 , -0.041 , -3*M_PI/4, -1.047 ,  1.047 , 0         , 29,     sAA  , sBB  , -1   , "TH5" ,  // 30
  0.0    ,  0.0   ,  M_PI/2  ,  M_PI/2,  M_PI/2, M_PI/2    , 30,     0    , 0    , -1   , "TH-Bridge4" ,  // 31
  0.0    ,  0.0   ,  0.0     ,  0.0   ,  1.222 , 1.1       , 31,     0    , 0    , -1   , "TH4" ,  // 32
  -0.038 ,  0.0   ,  0.0     , -0.209 ,  0.209 , M_PI/12   , 32,     sCC  , 0.010, -1   , "TH3" ,  // 33
  0.0    ,  0.0   ,  M_PI/2  , -0.698 ,  0.698 , M_PI/12   , 33,     0    , 0    , -1   , "TH2" ,  // 34
  -0.032 ,  0.0   ,  0.0     , -0.262 ,  1.571 , M_PI/12   , 34,     sCC  , sCC  , -1   , "TH1" ,  // 35
  -0.0275,  0.0   ,  0.0     ,  0.0   ,  0.0   , 0         , 35,     sCC  , sCC  , sDD  , "TH-Tip" ,  // 36
  // additional joints
  0.0125 , 0.0    , 0.0      , 0.0    , 0.0    , 0         , 5 ,     0    , 0    , sDD  , "FF1.5"  ,  // 37
  0.0125 , 0.0    , 0.0      , 0.0    , 0.0    , 0         , 11,     0    , 0    , sDD  , "MF1.5"  ,  // 38
  0.0125 , 0.0    , 0.0      , 0.0    , 0.0    , 0         , 17,     0    , 0    , sDD  , "RF1.5"  ,  // 39
  0.0125 , 0.0    , 0.0      , 0.0    , 0.0    , 0         , 26,     0    , 0    , -1   , "LF1.5"  ,  // 40

};

} // namespace grasp::robots