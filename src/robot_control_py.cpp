#include <pybind11/pybind11.h>
#include <pybind11/stl.h> 
#include <pybind11/eigen.h> 
#include "robot_kinematics.hpp" 

namespace py = pybind11;

PYBIND11_MODULE(robot_kinematics, m) {
    // Bind UR5Kinematics class
    py::class_<UR5Kinematics>(m, "UR5Kinematics")
        .def(py::init<const Vector6d&>(),  
             py::arg("joint_angles") = Vector6d::Zero())
        .def("getJointAngles", &UR5Kinematics::getJointAngles)
        .def("setJointAngles", &UR5Kinematics::setJointAngles)
        .def("forwardKinematics", &UR5Kinematics::forwardKinematics)
        .def("inverseKinematics", &UR5Kinematics::inverseKinematics)
        .def("findNearestIKSolution", &UR5Kinematics::findNearestIKSolution)
        .def("calculateNearestIKSolution", (Vector6d (UR5Kinematics::*)(const Matrix4d&, const Vector6d&)) &UR5Kinematics::calculateNearestIKSolution)
        .def("calculateNearestIKSolution", (Vector6d (UR5Kinematics::*)(const Matrix4d&)) &UR5Kinematics::calculateNearestIKSolution)
        .def_static("eulerAnglesToRotationMatrix", &UR5Kinematics::eulerAnglesToRotationMatrix)
        .def_static("rotationMatrixToEulerAngles", &UR5Kinematics::rotationMatrixToEulerAngles)
        .def_static("createTransformationMatrix", &UR5Kinematics::createTransformationMatrix)
        .def_static("extractRotationAndTranslation", &UR5Kinematics::extractRotationAndTranslation)
        .def_static("wrapToPi", &UR5Kinematics::wrapToPi);
}