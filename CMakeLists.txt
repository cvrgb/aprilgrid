cmake_minimum_required(VERSION 2.6)
project(aprilgrid)

###########################################################
# PACKAGE DEPENDENCY
###########################################################

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED)    
endif()
message(WARNING "OPENCV VERSION: " ${OpenCV_VERSION})

###########################################################
# COMPILER OPTIONS
###########################################################
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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer  -funroll-loops -Wno-sign-compare")

###########################################################
# OUTPUT PATH
###########################################################

set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin/)
set(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/lib/)


###########################################################
# PROJECT RELATED
###########################################################

include_directories(./apriltags/include  ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
aux_source_directory(./apriltags/src APRILGRID_SRCS)
set(APP_NAME_EXE test)    
add_executable(${APP_NAME_EXE} ${APP_NAME_EXE}.cpp ${APRILGRID_SRCS})
target_link_libraries(${APP_NAME_EXE} ${OpenCV_LIBS})
    
