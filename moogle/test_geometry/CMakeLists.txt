
add_executable(test_bbox bbox.cpp)
target_link_libraries(test_bbox Bahamut Ramuh Catch2::Catch2)

add_executable(test_levelset levelset.cpp)
target_link_libraries(test_levelset Bahamut Odin Ramuh Catch2::Catch2)
