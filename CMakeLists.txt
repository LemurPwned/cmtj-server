cmake_minimum_required(VERSION 3.18)
project(cmtj_server)

set(CMAKE_CXX_STANDARD 20)
find_package(Boost)
find_package(nlohmann_json 3.2.0 REQUIRED)

find_package(openssl REQUIRED)
if( OPENSSL_FOUND )

    include_directories(${OPENSSL_INCLUDE_DIR})
    message(STATUS "Using OpenSSL ${OPENSSL_VERSION}")
endif()


# HEADERS for the junction 
set(LOC_DIR "~/Documents/agh/pymtj/cmtj/")
include_directories(${LOC_DIR})

add_executable(cmtj_server
              main.cpp
              ${LOC_DIR}/junction.hpp
              ${LOC_DIR}/drivers.hpp
              ${LOC_DIR}/cvector.hpp)
target_link_libraries(cmtj_server PRIVATE nlohmann_json::nlohmann_json)
