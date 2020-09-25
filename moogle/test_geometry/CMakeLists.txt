
add_executable(test_bbox bbox.cpp)
target_link_libraries(test_bbox Bahamut Ramuh Catch2::Catch2)

add_executable(test_levelset levelset.cpp)
target_link_libraries(test_levelset Bahamut Odin Ramuh Catch2::Catch2)

add_executable(test_face_vertex_mesh face_vertex_mesh.cpp)
target_link_libraries(test_face_vertex_mesh Bahamut Odin Ramuh Catch2::Catch2)

add_executable(test_cube_tree cube_tree.cpp)
target_link_libraries(test_cube_tree Bahamut Odin Ramuh Catch2::Catch2)
