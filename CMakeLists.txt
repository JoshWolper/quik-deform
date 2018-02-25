cmake_minimum_required(VERSION 3.8)
project(QUIKDEFORM)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "-O3 ")

set(SOURCE_FILES main.cpp SVD.h SVD.cpp global.h QuikDeformer.h QuikDeformer.cpp Constraint.h StrainConstraint.h StrainConstraint.cpp)

add_executable(QUIKDEFORM ${SOURCE_FILES})
