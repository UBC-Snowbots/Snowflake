cmake_minimum_required(VERSION 3.4 FATAL_ERROR)

project(version1 CXX)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_library(SnowBotSerial SnowBotSerial.cpp)
target_link_libraries(SnowBotSerial -L/usr/lib/x86_64-linux-gnu/ -lserial -lpthread)
target_include_directories(SnowBotSerial PUBLIC .)

add_executable(testMain testMain.cpp)
target_link_libraries(testMain SnowBotSerial)
