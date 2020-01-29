cmake_minimum_required(VERSION 2.8.3)
project(codegen_admm)

set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
  # Options: Debug, Release, MinSizeRel, RelWithDebInfo
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release")
endif()

file(GLOB cxsparse_srcs "ADMMGainDesign3D/CXSparse/Source/*.c" "ADMMGainDesign3D/CXSparse/CXSparseSupport/*.c")
set_source_files_properties(${cxsparse_srcs} PROPERTIES LANGUAGE C)
file(GLOB admm_srcs "ADMMGainDesign3D/*.cpp")
set_source_files_properties(${admm_srcs} PROPERTIES LANGUAGE CXX)

add_library(${PROJECT_NAME} ${cxsparse_srcs} ${admm_srcs})
target_compile_definitions(${PROJECT_NAME} PRIVATE CS_COMPLEX)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  ADMMGainDesign3D
  PRIVATE
  ADMMGainDesign3D/CXSparse/Include
  ADMMGainDesign3D/CXSparse/include
  ADMMGainDesign3D/CXSparse/CXSparseSupport
)