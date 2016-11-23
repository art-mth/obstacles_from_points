cmake_minimum_required(VERSION 2.8.12)

set(SOURCES
    "src/obstacles_from_points.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/obstacles_from_points.h"
)

#set compiler flags
if((${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang") OR (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"))
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wreturn-type -Wpedantic ")
endif()

include_directories("include")

add_library(obstacles_from_points MODULE ${SOURCES} ${HEADERS})
target_link_libraries(obstacles_from_points PRIVATE lmscore)
