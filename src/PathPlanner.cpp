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
typedef PlannerResult<RRT::RRTMetaData> PRRRT;
typedef MetaData<RRT::RRTMetaData> MDRRT;

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
  RobotConfig start(x_start, y_start);
  RobotConfig end(x_end, y_end);
  planPath(start, end, grid_x_max, grid_y_max);
}

PRRRT callRRT2(RobotConfig start, RobotConfig end, float grid_x_max,
               float grid_y_max) {
  RRT rrt(start, end, grid_x_max, grid_y_max);
  rrt.runRRT();
  rrt.printPath();
  PRRRT result;
  result.path = rrt.returnPath();
  result.metadata.data = rrt.getMetaData();
  return result;
}

PYBIND11_MODULE(PathPlanner, m) {
  m.doc() =
      "A Path Planning Module written in C++";  // optional module docstring

  m.def("add", &add, "A function that adds two numbers");
  m.def("callRRT", &callRRT, "A function that calls RRT");
  m.def("callRRT2", &callRRT2, "A function that calls RRT2");
  m.def("planPath", &planPath, "A function that plans path");
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

  py::class_<AlgorithmData>(m, "AlgorithmData")
      .def(py::init<>())
      .def_readwrite("AlgorithmName", &AlgorithmData::AlgorithmName)
      .def_readwrite("runtime", &AlgorithmData::runtime);

  py::class_<MDRRT>(m, "MetaData")
      .def(py::init<>())
      .def_readwrite("AlgorithmData", &MDRRT::algorithm_data)
      .def_readwrite("data", &MDRRT::data)
      .def("__repr__", [](const MDRRT& data) { return data.toStream(); });

  py::class_<PRRRT>(m, "PlannerResult")
      .def(py::init<>())
      .def_readwrite("path", &PRRRT::path)
      .def_readwrite("metadata", &PRRRT::metadata);
}
