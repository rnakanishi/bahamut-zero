cmake_minimum_required(VERSION 3.1)
project(GARUDA 
        LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

file(GLOB HEADERS 
${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
${CMAKE_CURRENT_SOURCE_DIR}/imgui/*.hpp
${CMAKE_CURRENT_SOURCE_DIR}/imgui/*.h
)
file(GLOB SOURCES 
${CMAKE_CURRENT_SOURCE_DIR}/geometry/*.cpp
)

add_library(Garuda STATIC ${SOURCES} ${HEADERS})

add_dependencies(Garuda Bahamut)
target_link_libraries(Garuda PUBLIC Bahamut)
set_target_properties(Garuda PROPERTIES
        OUTPUT_NAME "Garuda"
        FOLDER "GARUDA")
target_include_directories(Garuda PUBLIC ${GARUDA_SOURCE_DIR})
