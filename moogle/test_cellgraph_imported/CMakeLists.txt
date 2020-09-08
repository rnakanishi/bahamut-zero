
add_executable(test_cGraphLoad import_cellgraph.cpp cellgraph_temp.cpp)
target_link_libraries(test_cGraphLoad Bahamut Ramuh Odin Catch2::Catch2)

