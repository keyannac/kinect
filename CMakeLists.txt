cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(test2)


set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED TRUE) 
set (CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})



set (OpenCV_DIR "../opencv/build")
find_package(OpenCV REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/libfreenect/cmake_modules")

include_directories(${libusb-1.0_INCLUDE_DIR})
find_package(libusb-1.0 REQUIRED)



add_executable(test2 test2.cpp)
target_link_libraries(test2 ${libusb-1.0} ${OpenCV_LIBS})

SET(COMPILE_FLAGS "-fPIC -g -Wall `pkg-config --cflags --libs opencv` -std=c++11")
add_definitions(${COMPILE_FLAGS})
