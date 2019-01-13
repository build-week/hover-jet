cmake_minimum_required(VERSION 2.8)
project(hover_jet)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/third_party")

# Crossguid
FIND_PACKAGE(crossguid REQUIRED)

# CMakeLists
find_package(OpenCV REQUIRED)

find_package(Boost 1.65 REQUIRED COMPONENTS wserialization system)
message(baboost "${Boost_LIBRARIES} ${Boost_LIBRARY_DIR}")
message("Foumd" Boost_FOUND)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/run)

# Get yaml-cpp configured
find_package(yaml-cpp REQUIRED)

# TODO: Make this run whenever make is run
execute_process(
    COMMAND pymake -v success -p ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE ret
)
if(ret EQUAL "1")
    message(FATAL_ERROR "Could not run pymake; Suggested solutions:
        Are you running in the docker image?
        Have you run `sudo pip install generate-cmake`?
        Talk to Ben or Jake if you expect this to work.
    ")
endif()

set(CMAKE_CXX_COMPILER "g++-7")
set(CMAKE_CXX_FLAGS "--std=c++17 -g -O1 -fno-omit-frame-pointer -Wall -Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")


# Wait! Don't edit this file! Use pymake of updating this cmake.
# It will automatically generate CMake files for your code.
include_directories("third_party/experiments/third_party/Sophus")
include_directories("third_party/experiments/third_party/eigen3")
include_directories("third_party/experiments")
add_subdirectory(third_party)
add_subdirectory(tmp)
