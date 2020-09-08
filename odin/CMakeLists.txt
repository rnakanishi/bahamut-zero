cmake_minimum_required(VERSION 3.1)
project(ODIN 
        LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

file(GLOB HEADERS 
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/interpolation/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/blas/*.hpp
        )
file(GLOB SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/interpolation/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/blas/*.cpp
        )

# include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_library(Odin STATIC ${SOURCES} ${HEADERS})
target_link_libraries(Odin PUBLIC Bahamut)
set_target_properties(Odin PROPERTIES
        OUTPUT_NAME "Odin"
        FOLDER "ODIN")
target_include_directories(Odin PUBLIC ${ODIN_SOURCE_DIR})
