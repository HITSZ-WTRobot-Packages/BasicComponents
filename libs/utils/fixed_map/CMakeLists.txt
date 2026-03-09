add_library(__libs_FixedMap INTERFACE)

target_include_directories(__libs_FixedMap INTERFACE
        .
)

# link dependencies if any


# alias for external use
add_library(libs::FixedMap ALIAS __libs_FixedMap)
