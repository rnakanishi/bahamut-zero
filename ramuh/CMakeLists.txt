cmake_minimum_required(VERSION 3.1)
project(RAMUH 
        LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

file(GLOB HEADERS 
${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
${CMAKE_CURRENT_SOURCE_DIR}/geometry/*.hpp
${CMAKE_CURRENT_SOURCE_DIR}/grid/*.hpp
${CMAKE_CURRENT_SOURCE_DIR}/particle/*.hpp
${CMAKE_CURRENT_SOURCE_DIR}/ramuh_utils/*.hpp
)
file(GLOB SOURCES 
${CMAKE_CURRENT_SOURCE_DIR}/geometry/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/grid/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/particle/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/ramuh_utils/*.cpp
)

# include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_library(Ramuh STATIC ${SOURCES} ${HEADERS})

add_dependencies(Ramuh Bahamut)
add_dependencies(Ramuh tinyobjloader)

target_link_libraries(Ramuh PUBLIC Bahamut tinyobjloader)
set_target_properties(Ramuh PROPERTIES
        OUTPUT_NAME "Ramuh"
        FOLDER "RAMUH")
target_include_directories(Ramuh PUBLIC ${RAMUH_SOURCE_DIR})
