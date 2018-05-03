# project
cmake_minimum_required (VERSION 3.2)
project (libviso2)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS thread)

# directories
set (LIBVISO2_SRC_DIR src/lib)

# include directory
include_directories("${LIBVISO2_SRC_DIR}" ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
#message(STATUS ${OpenCV_INCLUDE_DIRS})
# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

# sources
FILE(GLOB LIBVISO2_SRC_FILES "src/lib/*.cpp")

# make release version
set(CMAKE_BUILD_TYPE Debug)


add_library(viso2 ${LIBVISO2_SRC_FILES})
target_link_libraries (viso2 png ${Boost_LIBRARIES})

# demo program
add_executable(viso2_demo src/demo/demo.cpp)
target_link_libraries (viso2_demo viso2)

add_executable(result_viso src/result_viso/result_viso.cpp)
target_link_libraries (result_viso viso2 ${OpenCV_LIBRARIES})
#message(STATUS ${OpenCV_LIBRARIES})

message(STATUS ${CMAKE_BUILD_TYPE})

