#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "darwin.h"
#include "PathPlanner3D.h"

namespace py = pybind11;

PYBIND11_MODULE(arm_robot_cpp, m) {
    m.doc() = "arm robot cpp module for python binding";

    py::class_<arm_robot::Human>(m, "Human")
        .def(py::init<const std::vector<std::vector<double>>&,
                     const std::vector<std::vector<double>>&,
                     const std::vector<std::vector<double>>&,
                     const std::vector<std::vector<double>>&,
                     const Eigen::MatrixXd&,
                     const Eigen::MatrixXd&>(),
             py::arg("body_dh"), 
             py::arg("left_hand_dh"), 
             py::arg("right_hand_dh"),
             py::arg("head_dh"),
             py::arg("theta_limit_input") = Eigen::MatrixXd(),
             py::arg("base_offset") = Eigen::MatrixXd())
        .def("flash_theta", &arm_robot::Human::flashTheta)
        .def("get_all_tf", &arm_robot::Human::getAllTf)
        .def("get_eef", &arm_robot::Human::getEef)
        .def("get_num_joints", &arm_robot::Human::getNumJoints)
        .def("get_part_base", &arm_robot::Human::getPartBase)
        .def("get_eef_by_name", &arm_robot::Human::getEefByName)
        .def("get_all_theta", &arm_robot::Human::getAllTheta)
        .def("control_part", &arm_robot::Human::controlPart,
             py::arg("aim_mat"), 
             py::arg("part_name") = std::string("body"), 
             py::arg("ignore_angle") = false,
             py::arg("random_flag") = false,
             py::arg("q0") = std::vector<double>())
        .def("control_hand", &arm_robot::Human::controlHand,
             py::arg("aim_mat"), 
             py::arg("hand_name") = std::string("left"), 
             py::arg("ignore_angle") = false,
             py::arg("random_flag") = false,
             py::arg("q0") = std::vector<double>())
        .def("control_part_v", &arm_robot::Human::controlPartV,
             py::arg("aim_v"), 
             py::arg("part_name") = std::string("body"));

    // PathPlanner3D binding
    py::class_<PathPlanner3D>(m, "PathPlanner3D")
        .def(py::init<int, int, int, double, double, double, double>(),
             py::arg("width"), py::arg("height"), py::arg("depth"), 
             py::arg("resolution"), py::arg("origin_x") = 0.0, 
             py::arg("origin_y") = 0.0, py::arg("origin_z") = 0.0)
        .def("planWithObstacles", &PathPlanner3D::planWithObstacles,
             py::arg("start"), py::arg("goal"), py::arg("obstacles"), 
             py::arg("useSmoothing") = true);
}