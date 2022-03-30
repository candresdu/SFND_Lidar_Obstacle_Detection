cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(submission)

find_package(PCL 1.11 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories("include")
add_executable (submission src/submission.cpp src/render.cpp)
target_link_libraries (submission ${PCL_LIBRARIES})
target_include_directories(submission PUBLIC ${CMAKE_SOURCE_DIR}/include)




