cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 14)
project(crooked)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Boost REQUIRED COMPONENTS system)

find_package(ur_rtde REQUIRED)


file(GLOB SOURCES
    *.h
    *.cpp
)

add_executable(crooked ${SOURCES})
FILE(COPY shuttle-table.wav DESTINATION "${CMAKE_BINARY_DIR}")

target_include_directories(crooked PRIVATE ${Boost_INCLUDE_DIRS})

target_include_directories(crooked PRIVATE ${ur_rtde_INCLUDE_DIRS})
target_include_directories(crooked PRIVATE /usr/local/include/ur_rtde/)
target_link_libraries(crooked PRIVATE ur_rtde::rtde)

find_package(glm REQUIRED)
include_directories(${GLM_INCLUDE_DIRS})
link_directories(${GLM_LIBRARY_DIRS})
add_definitions(${GLM_DEFINITIONS})

message(STATUS "d = ${Boost_INCLUDE_DIRS} and ${ur_rtde_INCLUDE_DIRS}")
