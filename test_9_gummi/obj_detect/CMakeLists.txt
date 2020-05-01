cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 17)
project(obj_detect)

file(GLOB SOURCES
    src/*.h
    src/*.cpp
	dependencies/args/args.hxx
	dependencies/cpp-httplib/httplib.h
)

add_executable(obj_detect ${SOURCES})

target_include_directories(obj_detect PRIVATE dependencies/args )
target_include_directories(obj_detect PRIVATE dependencies/cpp-httplib)


find_package( OpenCV REQUIRED )
target_include_directories(obj_detect PRIVATE ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( obj_detect ${OpenCV_LIBS} )


configure_file(Info.plist ${CMAKE_CURRENT_BINARY_DIR}/Info.plist COPYONLY)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCES})