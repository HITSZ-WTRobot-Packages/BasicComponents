add_library(__libs_RingBuffer INTERFACE)

target_include_directories(__libs_RingBuffer INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any


# alias for external use
add_library(libs::RingBuffer ALIAS __libs_RingBuffer)
