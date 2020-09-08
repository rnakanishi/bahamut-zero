find_package(OpenVDB)

if(OpenVDB_FOUND)
  add_executable(openvdbHello hello.cpp)
  target_link_libraries(openvdbHello OpenVDB::openvdb)

  add_executable(openvdbPoints hello_points.cpp)
  target_link_libraries(openvdbPoints OpenVDB::openvdb)

  add_executable(openvdbGrid main.cpp)
  target_link_libraries(openvdbGrid OpenVDB::openvdb)
else()
  message("Could not find OpenVDB. Skipping OpenVDB tests.")
endif()