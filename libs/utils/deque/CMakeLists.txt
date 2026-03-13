add_library(__libs_Deque INTERFACE)

target_include_directories(__libs_Deque INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any


# alias for external use
add_library(libs::Deque ALIAS __libs_Deque)
