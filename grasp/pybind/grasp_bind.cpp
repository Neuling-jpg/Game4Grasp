#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/embed.h>
#include <iostream>

void grasp_cpp(const std::string& robot_name, 
        const std::string& object_name, 
        const std::string& log_dir, 
        int verbose) {

    std::vector<double> theta_solution;
    
    double ee_penalty, colli_penalty;
        
    // grasp::Grasp(robot_name, object_name, 
    //             log_dir, theta_solution, 
    //             ee_penalty, colli_penalty, 
    //             verbose);
    
    return ;
};

namespace py = pybind11;

PYBIND11_MODULE(grasp, m) {
    m.def("grasp", &grasp_cpp, 
          py::arg("robot_name"), 
          py::arg("object_name"),
          py::arg("log_dir"),
          py::arg("verbose"));
}
