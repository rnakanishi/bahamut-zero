
cmake_minimum_required(VERSION 3.1)
project(MOOGLE LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(Catch2 REQUIRED)

add_subdirectory(test_blas)
add_subdirectory(test_geometry)
add_subdirectory(test_cellgraph_imported)
add_subdirectory(test_particles)
add_subdirectory(test_grid)
add_subdirectory(test_houdini)
add_subdirectory(test_tbb)