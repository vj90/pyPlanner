#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <iostream>
#include <sstream>
#include <vector>

#include "PlannerResult.h"
#include "RRT.h"
#include "RobotConfig.h"
#include "helper.h"
namespace py = pybind11;

int add(int i, int j) {
  printMeow();
  return i + j;
}

void callRRT() {
  float x_start = 0;
  float y_start = 0;
  float x_end = 100;
  float y_end = 100;
  float grid_x_max = 100;
  float grid_y_max = 100;
  planPath(x_start, y_start, x_end, y_end, grid_x_max, grid_y_max);
}

PlannerResult callRRT2(RobotConfig start, RobotConfig end, float grid_x_max,
                       float grid_y_max) {
  RRT rrt(start, end, grid_x_max, grid_y_max);
  rrt.runRRT();
  rrt.printPath();
  PlannerResult result;
  result.path = rrt.returnPath();
  return result;
}

PYBIND11_MODULE(PathPlanner, m) {
  m.doc() =
      "A Path Planning Module written in C++";  // optional module docstring

  m.def("add", &add, "A function that adds two numbers");
  m.def("callRRT", &callRRT, "A function that calls RRT");
  m.def("callRRT2", &callRRT2, "A function that calls RRT2");
  py::class_<RobotConfig>(m, "RobotConfig")
      .def(py::init<const float, const float, const float>(), py::arg("x"),
           py::arg("y"), py::arg("theta") = NAN)
      .def(py::init<>())
      .def_readwrite("x", &RobotConfig::x)
      .def_readwrite("y", &RobotConfig::y)
      .def_readwrite("theta", &RobotConfig::theta)
      .def("__repr__", [](const RobotConfig& a) {
        std::ostringstream stream;
        stream << "[x = "
               << std::to_string(a.x) + ", y = " + std::to_string(a.y) +
                      ", theta =  " + std::to_string(a.theta)
               << "]";
        return stream.str();
      });

  py::class_<MetaData>(m, "MetaData")
      .def(py::init<>())
      .def_readwrite("AlgorithmName", &MetaData::AlgorithmName)
      .def_readwrite("runtime", &MetaData::runtime)
      .def_readwrite("all_nodes", &MetaData::all_nodes)
      .def("__repr__", [](const MetaData& data) {
        std::ostringstream stream;
        stream << "[Algorithm Name: " << data.AlgorithmName
               << ", Runtime: " << data.runtime;
        if (data.all_nodes.size() > 0) {
          stream << ", All Nodes: available]";
        } else {
          stream << ", All Nodes: not available]";
          // TODO fix this
          return stream.str();
        }
      });

  py::class_<PlannerResult>(m, "PlannerResult")
      .def(py::init<>())
      .def_readwrite("path", &PlannerResult::path)
      .def_readwrite("metadata", &PlannerResult::metadata);
}
