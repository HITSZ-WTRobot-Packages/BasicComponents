cmake_minimum_required(VERSION 3.16)
project(BasicComponents LANGUAGES C CXX)

# set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(bsp/can_driver)
add_subdirectory(bsp/gpio_driver)

add_subdirectory(libs/control)
add_subdirectory(libs/utils)
add_subdirectory(libs/math)
add_subdirectory(libs/concurrency)

add_subdirectory(services/watchdog)

add_subdirectory(protocol/UartRxSync)

add_subdirectory(utils)