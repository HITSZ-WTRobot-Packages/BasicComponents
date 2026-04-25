add_library(ServicesI2CUpdateManager STATIC
    "./I2CDevice.cpp"
    "./I2CUpdateManager.cpp"
)

target_include_directories(ServicesI2CUpdateManager
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(ServicesI2CUpdateManager PUBLIC stm32cubemx)
target_link_libraries(ServicesI2CUpdateManager PUBLIC bsp::I2CDriver)

# alias for external use
add_library(services::I2CUpdateManager ALIAS ServicesI2CUpdateManager)
