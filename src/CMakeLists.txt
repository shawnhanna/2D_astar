cmake_minimum_required(VERSION 2.8)

project(simple_path_planner)

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

# Create an AStar search library. Only used in simplePathPlanner,
#    but I'm showing how it's done for future use
add_library(astar AStar.cpp)
add_library(node Node.cpp)

# add the executable
add_executable(simple_path_planner SimplePathPlanner.cpp)

add_dependencies(simple_path_planner astar)
add_dependencies(astar node)

target_link_libraries( node ${OpenCV_LIBS} )
target_link_libraries( astar node ${OpenCV_LIBS} )
target_link_libraries( simple_path_planner astar ${OpenCV_LIBS} )
