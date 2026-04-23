add_library(BspI2CDriver STATIC
    "./I2CBusDMA.cpp"
)

target_include_directories(BspI2CDriver
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(BspI2CDriver
    PUBLIC
    stm32cubemx
)

add_library(bsp::I2CDriver ALIAS BspI2CDriver)
