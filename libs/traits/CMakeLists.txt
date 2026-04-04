add_library(___traits INTERFACE)

target_include_directories(___traits INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any


# alias for external use
add_library(traits ALIAS ___traits)
