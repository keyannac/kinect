cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(test2)


set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED TRUE) 
set (CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
set (CMAKE_MODULE_PATH "/libfreenect/cmake_modules")
find_package(PCL 1.2 REQUIRED)
find_package(libfreenect REQUIRED)

set (OpenCV_DIR "../opencv/build")
find_package(OpenCV REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/libfreenect/cmake_modules")
include(FindOS)
include(SetupDirectories)
include_directories(${libusb-1.0_INCLUDE_DIR})
find_package(libusb-1.0 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${FREENECT_INCLUDE_DIR} ${libusb-1.0_INCLUDE_DIR} )
link_directories(${PCL_LIBRARY_DIRS} ${libfreenect_LIBRARY_DIR})
add_definitions(${PCL_DEFINITIONS})

add_executable(test2 test2.cpp)
target_link_libraries(test2 ${PCL_LIBRARIES} ${FREENECT_LIBRARIES} ${libusb-1.0} ${OpenCV_LIBS})

SET(COMPILE_FLAGS "-fPIC -g -Wall `pkg-config --cflags --libs opencv` -std=c++11")
add_definitions(${COMPILE_FLAGS})
