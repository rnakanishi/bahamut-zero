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
${CMAKE_CURRENT_SOURCE_DIR}/garuda_utils/*.hpp
${CMAKE_CURRENT_SOURCE_DIR}/scene/*.hpp
)
file(GLOB SOURCES 
${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/renderable/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/garuda_utils/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/scene/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/imgui/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/glad/*.c
)

add_library(Garuda STATIC ${SOURCES} ${HEADERS})

find_package(glfw3 REQUIRED)
if(glfw3_FOUND)
  message("-- Found GLFW3")
endif()
set(OpenGL_GL_PREFERENCE LEGACY)

add_dependencies(Garuda glfw)
add_dependencies(Garuda Bahamut Bismarck)
add_dependencies(Garuda tinyobjloader)

target_link_libraries(Garuda PUBLIC Bahamut Bismarck ${GLFW_LIBRARIES} tinyobjloader OpenGL glfw dl)
set_target_properties(Garuda PROPERTIES
        OUTPUT_NAME "Garuda"
        FOLDER "GARUDA")
target_include_directories(Garuda PUBLIC ${GARUDA_SOURCE_DIR})
