#include <pybind11/pybind11.h>

#include "RRT.h"
#include "helper.h"

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

PYBIND11_MODULE(example, m) {
  m.doc() = "pybind11 example plugin";  // optional module docstring

  m.def("add", &add, "A function that adds two numbers");
  m.def("callRRT", &callRRT, "A function that calls RRT");
}
