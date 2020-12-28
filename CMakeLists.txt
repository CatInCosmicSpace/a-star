cmake_minimum_required(VERSION 3.17)
project(a-star)

set(CMAKE_CXX_STANDARD 14)

include_directories("include")


set(SOURCES sources/grid.cpp sources/maze.cpp)
set(HEADERS include/grid.h include/maze.h include/parallel.cpp include/parallel.h)

add_executable(a-star main.cpp ${SOURCES} ${HEADERS})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(a-star PRIVATE Threads::Threads)