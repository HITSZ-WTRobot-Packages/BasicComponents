add_library(__libs_Concurrency INTERFACE)

target_include_directories(__libs_Concurrency INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any


# alias for external use
add_library(libs::Concurrency ALIAS __libs_Concurrency)
