cmake_minimum_required(VERSION 3.18)
project(cmtj_server)

set(CMAKE_CXX_STANDARD 20)


######## FFTW DOWNLOADS ########

# download FFTW
configure_file(downloadFindFFTW.cmake.in findFFTW-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/findFFTW-download )
if(result)
    message(FATAL_ERROR "CMake step for findFFTW failed: ${result}")
    else()
    message("CMake step for findFFTW completed (${result}).")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/findFFTW-download )
if(result)
    message(FATAL_ERROR "Build step for findFFTW failed: ${result}")
endif()

set(findFFTW_DIR ${CMAKE_CURRENT_BINARY_DIR}/findFFTW-src)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${findFFTW_DIR}")



find_package(Boost REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package (Threads)
find_package(fmt)
find_package(FFTW REQUIRED)
find_package (leveldb REQUIRED)
find_package(spdlog REQUIRED)
# find_package(OpenSSL REQUIRED)
# if( OPENSSL_FOUND )
#     include_directories(${OPENSSL_INCLUDE_DIR})
#     message(STATUS "Using OpenSSL ${OPENSSL_VERSION}")
# endif()


# HEADERS for the junction 
set(LOC_DIR "~/Documents/agh/pymtj/cmtj/core")
include_directories(${LOC_DIR})

add_executable(cmtj_server
              main.cpp
              ${FFTW_INCLUDE_DIRS}
              ${LOC_DIR}/junction.hpp
              ${LOC_DIR}/drivers.hpp
              ${LOC_DIR}/cvector.hpp)
target_link_libraries(cmtj_server nlohmann_json::nlohmann_json fmt::fmt leveldb::leveldb ${FFTW_DOUBLE_LIB} ${FFTW_DOUBLE_THREADS_LIB})
 