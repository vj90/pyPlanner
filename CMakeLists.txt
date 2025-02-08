cmake_minimum_required(VERSION 3.15)

project(PathPlanner)


# The c++ standard. Setting required to True will make it binding
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#set(pybind11_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pyenv/lib/python3.10/site-packages/pybind11/include)
#find_package(pybind11 REQUIRED)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

add_subdirectory(src)