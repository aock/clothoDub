cmake_minimum_required(VERSION 2.8)
project(CLOTHO_DUB)

set_property(GLOBAL PROPERTY USE_FOLDERS On)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include_directories(include)

# C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    message(STATUS "Using C++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

if(CMAKE_COMPILER_IS_GNUCC)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -g")
    add_compile_options(-O0 -g)
endif(CMAKE_COMPILER_IS_GNUCC)

# EXTERNAL

add_subdirectory(ext/Dubins-Curves)
include_directories(ext/Dubins-Curves/include)
add_subdirectory(ext/g1fitting)
include_directories(ext/g1fitting)

# EXECUTABLES

set(CLOTHO_DUB_SRCS
    src/tools/clothodub_test.cpp
    src/libclothodub/clothodub.cpp
)

set(CLOTHO_DUB_DEPS
    g1fitting
    dubins
)

add_executable(clothodub_test ${CLOTHO_DUB_SRCS})
