project(Lane_Change)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/vehicle.cpp src/vehicle.h)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/opt/homebrew/Cellar/openssl@3/3.0.5/include)
include_directories(/opt/homebrew/Cellar/libuv/1.44.2/include)
link_directories(/usr/local/lib)
link_directories(/opt/homebrew/Cellar/openssl@3/3.0.5/lib)
link_directories(/opt/homebrew/Cellar/libuv/1.44.2/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(lane_change ${sources})

target_link_libraries(lane_change z ssl uv uWS)
