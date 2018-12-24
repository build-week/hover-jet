cmake_minimum_required(VERSION 2.8)
project(hover_jet)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/run)

# TODO: Make this run whenever make is run
execute_process(
    COMMAND pymake -v success -p ${CMAKE_SOURCE_DIR}
)
if(ret EQUAL "1")
    message(FATAL_ERROR "Could not run pymake; Have you run `sudo pip install generate-cmake`?")
endif()

set(CMAKE_CXX_COMPILER "g++-7")
set(CMAKE_CXX_FLAGS "--std=c++17 -g -O1 -fno-omit-frame-pointer -Wall -Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")


add_subdirectory(third_party)
add_subdirectory(tmp)
