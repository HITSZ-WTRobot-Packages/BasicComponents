add_library(BspCANDriver STATIC
    "./can_driver.c"
)

target_include_directories(BspCANDriver
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(BspCANDriver PUBLIC stm32cubemx)

# alias for external use
add_library(bsp::CANDriver ALIAS BspCANDriver)
