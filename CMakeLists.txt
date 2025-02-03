cmake_minimum_required(VERSION 3.15)

project(Example)

#set(pybind11_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pyenv/lib/python3.10/site-packages/pybind11/include)
#find_package(pybind11 REQUIRED)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

add_library(helper STATIC src/helper.cpp)
# The following is required because helper is built as a static library, which 
# will later be linked into a shared library. Thus, helper must be built with 
# position independent code enabled (-fPIC flag). Shared libraries require 
# Position-Independent Code so that they can be loaded at different memory addresses.
set_target_properties(helper PROPERTIES POSITION_INDEPENDENT_CODE ON)


set(SOURCES src/PathPlanner.cpp src/RRT.cpp)
pybind11_add_module(PathPlanner ${SOURCES})

target_link_libraries(PathPlanner PRIVATE helper)