cmake_minimum_required(VERSION 3.16)
project(ecal_udp_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ecal REQUIRED)

add_executable(ecal_udp_bridge main.cpp)
target_link_libraries(ecal_udp_bridge
    ecal::core
    pthread
)

install(TARGETS ecal_udp_bridge DESTINATION bin)
