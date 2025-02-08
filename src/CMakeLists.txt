set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

add_library(helper STATIC helper.cpp)
# The following is required because helper is built as a static library, which 
# will later be linked into a shared library. Thus, helper must be built with 
# position independent code enabled (-fPIC flag). Shared libraries require 
# Position-Independent Code so that they can be loaded at different memory addresses.
set_target_properties(helper PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_subdirectory(planning)


set(SOURCES PathPlanner.cpp)
pybind11_add_module(PathPlanner ${SOURCES})

target_link_libraries(PathPlanner PRIVATE helper planning)

target_include_directories(PathPlanner PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/planning")

set_target_properties( PathPlanner
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)