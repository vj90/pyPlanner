cmake_minimum_required(VERSION 3.15)

project(Example)

#set(pybind11_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pyenv/lib/python3.10/site-packages/pybind11/include)
#find_package(pybind11 REQUIRED)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)



set(SOURCES src/example.cpp src/helper.cpp src/RRT.cpp)
pybind11_add_module(example ${SOURCES})