cmake_minimum_required(VERSION 3.10)
project(WallerLocal)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(waller
    main.cpp
    environment.cpp
)

target_include_directories(waller PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})