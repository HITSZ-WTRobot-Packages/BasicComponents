add_library(BspCANDriver STATIC
    "./can_driver.cpp"
)

target_include_directories(BspCANDriver
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(BspCANDriver PUBLIC stm32cubemx)
target_link_libraries(BspCANDriver PUBLIC libs::RingBuffer)
target_link_libraries(BspCANDriver PUBLIC utils)

# alias for external use
add_library(bsp::CANDriver ALIAS BspCANDriver)
