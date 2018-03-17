cmake_minimum_required(VERSION 3.8)
project(QUIKDEFORM)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "-O3 ")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(SOURCE_FILES main.cpp SVD.h SVD.cpp global.h QuikDeformer.h QuikDeformer.cpp Constraint.h StrainConstraint.h StrainConstraint.cpp PositionConstraint.h PositionConstraint.cpp GroundConstraint.cpp GroundConstraint.h)

add_executable(QUIKDEFORM ${SOURCE_FILES})
