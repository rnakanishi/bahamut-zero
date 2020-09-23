cmake_minimum_required(VERSION 3.1)
project(BISMARCK 
        LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

file(GLOB HEADERS 
${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.hpp
${CMAKE_CURRENT_SOURCE_DIR}/materials/*.hpp
${CMAKE_CURRENT_SOURCE_DIR}/glad/*.h
)
file(GLOB SOURCES 
${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/materials/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/glad/*.c
)

add_library(Bismarck STATIC ${SOURCES} ${HEADERS})

set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)
if(OpenGL_FOUND)
  message("-- Found OpenGL")
  message("-- Set GL_PREFERENCE to LEGACY")
endif()

add_dependencies(Bismarck Bahamut)
target_link_libraries(Bismarck PUBLIC Bahamut OpenGL dl)
set_target_properties(Bismarck PROPERTIES
        OUTPUT_NAME "Bismarck"
        FOLDER "BISMARCK")
target_include_directories(Bismarck PUBLIC ${BISMARCK_SOURCE_DIR})
