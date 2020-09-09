cmake_minimum_required(VERSION 3.6)
project(BAHAMUT_ZERO_LIB)
set(CMAKE_CXX_STANDARD 11)

if(APPLE)
    ## SET OPENMP LOCAL FILES   
    set(OPENMP_LIBRARIES "/usr/local/Cellar/libomp/9.0.1/lib")
    set(OPENMP_INCLUDES "/usr/local/Cellar/libomp/9.0.1/include")
    
    set(OPENVDB_INCLUDES "/usr/local/include/openvdb")

    # Threads 
    set(CMAKE_THREAD_LIBS_INIT "-lpthread")
    set(CMAKE_HAVE_THREADS_LIBRARY 1)
    set(CMAKE_USE_WIN32_THREADS_INIT 0)
    set(CMAKE_USE_PTHREADS_INIT 1)
    set(THREADS_PREFER_PTHREAD_FLAG ON)

    if(CMAKE_C_COMPILER_ID MATCHES "Clang")
        set(OpenMP_C "${CMAKE_C_COMPILER}")
        set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -lomp -Wno-unused-command-line-argument")
        set(OpenMP_C_LIB_NAMES "libomp" "libgomp" "libiomp5")
        set(OpenMP_libomp_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_libgomp_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_libiomp5_LIBRARY ${OpenMP_C_LIB_NAMES})
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set(OpenMP_CXX "${CMAKE_CXX_COMPILER}")
        set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -lomp -Wno-unused-command-line-argument")
        set(OpenMP_CXX_LIB_NAMES "libomp" "libgomp" "libiomp5")
        set(OpenMP_libomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
        set(OpenMP_libgomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
        set(OpenMP_libiomp5_LIBRARY ${OpenMP_CXX_LIB_NAMES})
    endif()
else()
    list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")

    if (CMAKE_COMPILER_IS_GNUCXX)
        set(DEBUG_FLAGS "-g -O0 -Wall  -Wextra --coverage -fopenmp")
        set(RELEASE_FLAGS "-O3 -fopenmp")
        set(CMAKE_EXE_LINKER_FLAGS "-lstdc++fs -fprofile-arcs -ftest-coverage")
    else (CMAKE_COMPILER_IS_GNUCXX)
        set(DEBUG_FLAGS "/Zi /EHsc /MDd /MP /std:c++latest")
        set(RELEASE_FLAGS "/EHsc /std:c++latest.")
    endif (CMAKE_COMPILER_IS_GNUCXX)
endif()

find_package(OpenMP REQUIRED)
if (OpenMP_FOUND)
    # include_directories("${OpenMP_INCLUDES}")
    # link_directories("${OpenMP_LIBRARIES}")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    # set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OpenMP_FOUND)

find_package(TBB REQUIRED)
if(TBB_FOUND)
    message("-- Found TBB lib")
endif()
find_package(Threads REQUIRED)

find_package(OpenVDB)
if(OpenVDB_FOUND)
    message("-- Found OpenVDB")
endif()

find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    message("-- Found Eigen3")
    include_directories(${EIGEN3_INCLUDE_DIR})
else()
    message("-- Building local Eigen3")
    set(EIGEN3_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/eigen3)
    include_directories(${EIGEN3_INCLUDE_DIR})
endif()

list( APPEND CMAKE_PREFIX_PATH "/opt/hfs18.0/toolkit/cmake" )
find_package( Houdini REQUIRED )
if(Houdini_FOUND)
    message("-- Found Houdini")
endif()

add_subdirectory(ramuh) # Geometric structures lib
add_subdirectory(bahamut) # General utility library
add_subdirectory(odin) # Mathematical/Blas library
add_subdirectory(moogle) # Tests library
add_subdirectory(garuda) # Graaphical interface library
add_subdirectory(alexander) # Deploying final appllications
