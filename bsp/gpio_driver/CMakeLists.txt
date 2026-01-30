add_library(BspGPIODriver STATIC
    "./gpio_driver.c"
)

target_include_directories(BspGPIODriver
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(BspGPIODriver PUBLIC stm32cubemx)

# alias for external use
add_library(bsp::GPIO_Driver ALIAS BspGPIODriver)
