cmake_minimum_required(VERSION 3.10)
project(WallerLocal)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

add_executable(waller
    main.cpp
    environment.cpp
    robot.cpp
)

target_include_directories(waller PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})