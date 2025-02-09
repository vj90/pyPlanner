add_library(planning STATIC Obstacle.cpp RRT.cpp AStar.cpp)
set_target_properties(planning PROPERTIES POSITION_INDEPENDENT_CODE ON)

