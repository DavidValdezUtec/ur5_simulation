// src/python_bindings.cpp

// ──────────────────────────────────────────────────────────────
// Pybind11 headers - Enable interoperability between C++ and Python
// ──────────────────────────────────────────────────────────────
#include <pybind11/pybind11.h>       // Core pybind11 functionality
#include <pybind11/stl.h>            // Support for binding STL containers like std::string
#include <pybind11/functional.h>     // Support for binding std::function and lambdas

// ──────────────────────────────────────────────────────────────
// Qt headers - Needed here because launch_rviz depends on QWidget types
// ──────────────────────────────────────────────────────────────
#include <QWidget>                   // Required by embedded RViz QWidget references

// ──────────────────────────────────────────────────────────────
// Project headers
// ──────────────────────────────────────────────────────────────
#include "ur5_interfaz_panel/rviz_widget.hpp"  // Declares the launch_rviz function

namespace py = pybind11;  // Namespace alias for brevity

// ──────────────────────────────────────────────────────────────
// Python Module Definition
// ──────────────────────────────────────────────────────────────
/**
 * @brief Python binding for the RViz embedding function.
 *
 * This defines a Python module named `rviz_widget_py` that exposes the
 * `launch_rviz()` function. It allows PyQt5/PySide2 Python applications
 * to call the C++ function to embed RViz inside a Qt widget.
 */
PYBIND11_MODULE(rviz_widget_py, m) {
    m.def(
    "launch_rviz",
    &rviz_embed::launch_rviz,
    py::arg("parent_win_id"), 
    py::arg("urdf_path") = "", 
    py::arg("description_topic") = "/robot_description",
    py::arg("fixed_frame") = "world",
    "Launch RViz. Pass empty urdf_path for passive mode (external publishers)."
    );

    m.def(
        "add_robot",
        &rviz_embed::add_robot,
        py::arg("description_topic"),
        "Add another RobotModel display subscribing to the given topic"
    );

    m.def("cleanup_rviz", &rviz_embed::cleanup_rviz);
    m.def("_register_emergency_cleanup", []() {
        py::module_::import("atexit").attr("register")(
            py::cpp_function(&rviz_embed::emergency_cleanup)
        );
    });
}
