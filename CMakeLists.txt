cmake_minimum_required(VERSION 3.16)
project(BasicComponents LANGUAGES C CXX)

# set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(bsp/can_driver)

add_subdirectory(libs/pid_motor)
add_subdirectory(libs/pid_pd)
add_subdirectory(libs/mit_pd)
add_subdirectory(libs/printf)
add_subdirectory(libs/ring_buffer)

add_subdirectory(services/watchdog)

add_subdirectory(protocol/UartRxSync)

add_subdirectory(utils)