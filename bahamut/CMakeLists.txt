cmake_minimum_required(VERSION 3.1)
project(BAHAMUT 
        LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

file(GLOB HEADERS 
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/utils/*.hpp
        )
file(GLOB SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/utils/*.cpp
        )

# include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_library(Bahamut STATIC ${SOURCES} ${HEADERS})
set_target_properties(Bahamut PROPERTIES
        OUTPUT_NAME "Bahamut"
        FOLDER "BAHAMUT")
target_include_directories(Bahamut PUBLIC ${BAHAMUT_SOURCE_DIR})
