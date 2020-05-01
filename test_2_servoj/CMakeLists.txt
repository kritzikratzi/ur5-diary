cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 14)
project(testo)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Boost REQUIRED COMPONENTS system)


find_package(ur_rtde REQUIRED)

add_executable(testo main.cpp)
target_include_directories(testo PRIVATE ${Boost_INCLUDE_DIRS})
target_include_directories(testo PRIVATE ${ur_rtde_INCLUDE_DIRS})
target_include_directories(testo PRIVATE /usr/local/include/ur_rtde/)
target_link_libraries(testo PRIVATE ur_rtde::rtde)

message(STATUS "d = ${Boost_INCLUDE_DIRS} and ${ur_rtde_INCLUDE_DIRS}")
