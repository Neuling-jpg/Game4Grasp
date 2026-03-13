#include "grasp.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <random>

void saveThetaSolution(
                        const std::string& robot_name,
                        const std::vector<double>& base_trans,
                        const std::vector<double>& base_euler,
                        const std::vector<double>& s, 
                        const std::string& save_path) {
  
  if (base_trans.size() != 3) {
    std::cout << "[ERROR] base_trans.size() != 3, got size " << base_trans.size() << std::endl;
		throw std::runtime_error("[ERROR]");
  }
  if (base_euler.size() != 3) {
    std::cout << "[ERROR] base_euler.size() != 3, got size " << base_euler.size() << std::endl;
		throw std::runtime_error("[ERROR]");
  }

  std::vector<double> s_filtered;
  
  if (robot_name == "barrett") {
    s_filtered = {
      base_trans[0], base_trans[1], base_trans[2],
      base_euler[0], base_euler[2], base_euler[1], // base euler
      s[13], s[14], // J32, J33
      s[3], s[4], s[5], // J11, J12, J13
      s[8], s[9], s[10], // J21, J22, J23
    };
  }
  else if (robot_name == "allegro") {
    s_filtered = {
      base_trans[0], base_trans[1], base_trans[2],
      base_euler[0], base_euler[2], base_euler[1], // base euler
      s[2]+s[3], s[5]+s[6], s[7]+s[8], s[9]+s[10],  // 0.0, 1.0, 2.0, 3.0
      s[12]+s[13], s[15]+s[16], s[17]+s[18], s[19]+s[10],  // 4.0, 5.0, 6.0, 7.0
      s[23]+s[24], s[26]+s[27], s[28]+s[29], s[30]+s[31],  // 8.0, 9.0, 10.0, 11.0
      s[36], s[38]+s[39], s[41]+s[42], s[43]+s[44],  // 12.0, 13.0, 14.0, 15.0
    };
  }
  else if (robot_name == "leaphand") {
    s_filtered = {
      base_trans[0], base_trans[1], base_trans[2],
      base_euler[0], base_euler[2], base_euler[1], // base euler
      s[5]+s[6], s[3]+s[4], s[7]+s[8], s[9]+s[10],  // 0, 1, 2, 3
      s[15]+s[16], s[13]+s[14], s[17]+s[18], s[19]+s[10],  // 4, 5, 6, 7
      s[25]+s[26], s[23]+s[24], s[27]+s[28], s[29]+s[30],  // 8, 9, 10, 11
      s[34]+s[35], s[36]+s[37], s[39]+s[40], s[41]+s[42],  // 12, 13, 14, 15
    };
  }
  else if (robot_name == "shadowhand") {
    s_filtered = {
      base_trans[0], base_trans[1], base_trans[2],
      base_euler[0], base_euler[2], base_euler[1], // base euler
      s[0], s[1],  // WR2, WR1
      s[3], s[4], s[5], s[6],  // FF4, FF3, FF2, FF1
      s[9], s[10], s[11], s[12],  // MF4, MF3, MF2, MF1
      s[15], s[16], s[17], s[18],  // RF4, RF3, RF2, RF1
      s[21], s[24], s[25], s[26], s[27],  // LF5, LF4, LF3, LF2, LF1
      s[30], s[32], s[33], s[34], s[35],  // TH5, TH4, TH3, TH2, TH1
    };
  }
  else {
    std::cerr << "Got invalid robot name " << robot_name 
    << " not in [barrett, allegro, leaphand, shadowhand]" << std::endl;
    exit(0);
  }
  
  grasp::saveToFile(s_filtered, save_path);
} ;


int Grasp(
    const std::string& robot_name,
    const std::vector<double>& base_euler,
    const std::string& object_name,
    const std::string& log_dir,
    // std::vector<double>& ti,
    std::vector<double>& theta_solution,
    double& ee_penalty,
    double& colli_penalty,
    bool verbose,
    const std::string& ini_base_path
    ) {

  if (verbose) std::cout << "####### Grasp() Begin #######" << std::endl;
  
  // Enable Eigen's built-in parallelization
  Eigen::initParallel();  // Initialize parallelization
  Eigen::setNbThreads(4); // Set to number of CPU cores
  // TODO: ^ this is not functioning as expected
  
  grasp::GraspOpt hand(robot_name, verbose);

  std::vector<std::string> link_names = hand.GetLinkNames();

  hand.UpdateBaseEuler6D(base_euler);

  /////////////////////////////////////////////
  /////////////////////////////////////////////
  /////////////////////////////////////////////

  std::vector<std::vector<double>> object_pts;
  object_pts = grasp::readFromFile2D(log_dir + "/object_pc.txt");

  // initial base frame
  std::vector<std::vector<double>> base_ini;
  if (ini_base_path != "") {
    base_ini = grasp::readFromFile2D(ini_base_path);
    if (base_ini.size() == 4 && base_ini[0].size() == 4) {
       if (verbose) std::cout << "Loaded initial base from " << ini_base_path << std::endl;
    }
    else {
      std::cout << "[ERROR] Invalid initial base frame size from " << ini_base_path 
                << ", got size " << base_ini.size() << " x " << (base_ini.size()>0?base_ini[0].size():0) << std::endl;
      throw std::runtime_error("[ERROR]");
    }
  }
  else {
    if (verbose) std::cout << "No initial base path provided, using default." << std::endl;
  }
  
  hand.ComputeGrasp(theta_solution, object_pts, base_ini);
  
  ee_penalty = hand.ee_penalty;  // Get the penalty for the IK solution
  colli_penalty = hand.colli_penalty;  // Get the collision penalty (if any
  
  grasp::saveToFile(hand.adjusted_initial_pts_with_parent, log_dir + "/robot_pts_ini.txt");
  grasp::saveToFile(hand.optimized_pts_with_parent, log_dir + "/robot_pts_opt.txt");

  if (verbose) for (int i = 0; i < hand.num_joints; i++){
    std::cout << "[" << i << "] Joint " << link_names[i] 
    << " angle: " << theta_solution[i] << std::endl;
  }

  saveThetaSolution(robot_name, hand.base_trans, hand.base_euler_angle, theta_solution, 
                    log_dir + "/predict_q.txt");
  grasp::saveToFile(hand.GetBaseFrame(), log_dir + "/robot_base_mat.txt");

  Eigen::setNbThreads(0);
  
  if (verbose) std::cout << "####### Grasp() End #######" << std::endl;
  
  return 0;  // Return the joint angles as the solution
};

int main(int argc, char* argv[]) {
    std::string robot_name;
    std::string object_name;
    std::string log_dir;
    std::string verbose = "1";
    std::string ini_base_path = "";

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--robot_name" && i + 1 < argc) {
            robot_name = argv[++i];
        }
        else if (arg == "--object_name" && i + 1 < argc) {
            object_name = argv[++i];
        }
        else if (arg == "--log_dir" && i + 1 < argc) {
            log_dir = argv[++i];
        }
        else if (arg == "--verbose" && i + 1 < argc) {
            verbose = argv[++i];
        }
        else if (arg == "--ini_base_path" && i + 1 < argc) {
            ini_base_path = argv[++i];
        }
        else {
            std::cerr << "Invalid argument: " << arg << "\n"
                      << "Usage: " << argv[0] << " --robot_name <name> --object_name <name> --log_dir <path> [--verbose <0, 1>] [--ini_base_path <path>]\n";
            return 1;
        }
    }

    // Validate required arguments
    if (robot_name.empty() || object_name.empty() || log_dir.empty() || (verbose != "0" && verbose != "1")) {
        std::cerr << "Missing required arguments\n"
                  << "Usage: " << argv[0] << " --robot_name <name> --object_name <name> --log_dir <path> [--verbose <0, 1>] [--ini_base_path <path>]\n";
        return 1;
    }

    const bool verbose_flag = (verbose == "1") ;

    // Display parsed arguments
    if (verbose_flag) {
      std::cout << "Robot name: " << robot_name << std::endl;
      std::cout << "Object name: " << object_name << std::endl;
      std::cout << "Log directory: " << log_dir << std::endl;
      if (ini_base_path != "") std::cout << "Initial base path: " << ini_base_path << std::endl;
      else std::cout << "No initial base path provided, using default." << std::endl;
    }
  
    /////////////////////////////////////////////////////////////
    /////////////////////// main code ///////////////////////////
    /////////////////////////////////////////////////////////////

    std::vector<double> base_euler;
    std::vector<std::vector<double>> object_pts;
    // std::vector<double> theta_ini; 
    std::vector<double> theta_solution;
    double ee_penalty, colli_penalty;
    
    base_euler = {0, 0, 0, 0, 0, 0};
    
    // Modified function call with new parameters
    Grasp(robot_name, base_euler, object_name, log_dir, theta_solution, 
                          ee_penalty, colli_penalty, verbose_flag, ini_base_path);
    
    return 0;
} ;