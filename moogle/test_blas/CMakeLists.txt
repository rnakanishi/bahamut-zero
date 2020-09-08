
add_executable(test_rbf2 rbf2.cpp)
target_link_libraries(test_rbf2 Odin Catch2::Catch2)

add_executable(test_rbffd rbffd_test.cpp)
target_link_libraries(test_rbffd Odin Ramuh Catch2::Catch2)

add_executable(test_poisson poisson.cpp)
target_link_libraries(test_poisson Bahamut Odin Ramuh Catch2::Catch2)

add_executable(test_fd test_fd.cpp)
target_link_libraries(test_fd Bahamut Odin Ramuh Catch2::Catch2)

add_executable(test_laplacian_rbf laplacian_rbf.cpp)
target_link_libraries(test_laplacian_rbf Bahamut Odin Ramuh Catch2::Catch2)
