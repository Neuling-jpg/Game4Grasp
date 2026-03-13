cmake_minimum_required(VERSION 2.8.3)
project(Grasp)

if(MSVC)
  # Disable all warnings for MSVC
  add_compile_options(/W0)
else()
  # Disable all warnings for GCC/Clang
  add_compile_options(-w)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_options(-std=c++17)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Release)
file(GLOB_RECURSE ALL_HDRS 
  "include/*.hpp"
  )
file(GLOB_RECURSE ALL_SRCS "source/*.cpp")

# message("ALL_HDRS:", ${ALL_HDRS})

include_directories(DIRECTORY 
  include
  usr/local
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${pybind11_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(pybind11 REQUIRED)

add_library(${PROJECT_NAME} #SHARED
  ${ALL_SRCS}
)

set(test_cpp_dir "test/")
set(test_cpp_files "")

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_USE_OPENMP")

file(GLOB_RECURSE test_cpp_files "${test_cpp_dir}/*.cpp")
foreach(test_cpp_file ${test_cpp_files})
  get_filename_component(test_cpp_name ${test_cpp_file} NAME_WE)
  ADD_EXECUTABLE(${test_cpp_name} ${test_cpp_dir}/${test_cpp_name}.cpp)
  
  TARGET_LINK_LIBRARIES(${test_cpp_name} 
    ${PROJECT_NAME} 
    ${CMAKE_THREAD_LIBS_INIT}
    ${CERES_LIBRARIES}
  )
endforeach(test_cpp_file ${test_cpp_files})

pybind11_add_module(grasp #MODULE
    pybind/grasp_bind.cpp
)

# ADD_EXECUTABLE(graspopt grasp/grasp.cpp)
# target_compile_definitions(grasp PRIVATE EIGEN_DONT_PARALLELIZE=0)
TARGET_LINK_LIBRARIES(grasp 
  ${PROJECT_NAME} 
  ${CERES_LIBRARIES}
  pybind11::module
)

