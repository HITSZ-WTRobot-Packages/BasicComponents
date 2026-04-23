add_library(ServicesI2CUpdateManager STATIC
    "./I2CDevice.cpp"
    "./I2CUpdateManager.cpp"
)

target_include_directories(ServicesI2CUpdateManager
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(ServicesI2CUpdateManager
    PUBLIC
    stm32cubemx
    bsp::I2CDriver
)

add_library(services::I2CUpdateManager ALIAS ServicesI2CUpdateManager)
