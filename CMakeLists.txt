cmake_minimum_required(VERSION 3.17)
project(a-star)

set(CMAKE_CXX_STANDARD 14)

include_directories("include")

set(SOURCES sources/grid.cpp sources/maze.cpp)
set(HEADERS include/grid.h include/maze.h)

add_executable(a-star main.cpp ${SOURCES} ${HEADERS})
