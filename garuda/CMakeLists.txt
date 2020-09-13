cmake_minimum_required(VERSION 3.1)
project(GARUDA 
        LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

file(GLOB HEADERS 
${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
${CMAKE_CURRENT_SOURCE_DIR}/imgui/*.hpp
${CMAKE_CURRENT_SOURCE_DIR}/imgui/*.h
${CMAKE_CURRENT_SOURCE_DIR}/glad/*.h
${CMAKE_CURRENT_SOURCE_DIR}/renderable/*.hpp
)
file(GLOB SOURCES 
${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/renderable/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/imgui/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/glad/*.c
)

add_library(Garuda STATIC ${SOURCES} ${HEADERS})

find_package(glfw3 REQUIRED)
if(glfw3_FOUND)
  message("-- Found GLFW3")
endif()
set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)
if(OpenGL_FOUND)
  message("-- Found OpenGL")
  message("-- Set GL_PREFERENCE to LEGACY")
endif()

add_dependencies(Garuda glfw)

target_link_libraries(Garuda PUBLIC Bahamut Bismarck ${GLFW_LIBRARIES} OpenGL glfw dl)
add_dependencies(Garuda Bahamut Bismarck)
set_target_properties(Garuda PROPERTIES
        OUTPUT_NAME "Garuda"
        FOLDER "GARUDA")
target_include_directories(Garuda PUBLIC ${GARUDA_SOURCE_DIR})
