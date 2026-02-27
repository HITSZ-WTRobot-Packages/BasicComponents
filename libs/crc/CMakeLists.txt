add_library(__libs_CRC INTERFACE)

target_include_directories(__libs_CRC INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any


# alias for external use
add_library(libs::CRC ALIAS __libs_CRC)
