add_library(LibsPrintf STATIC
    "./printf.c"
)

target_include_directories(LibsPrintf
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any


# alias for external use
add_library(libs::printf ALIAS LibsPrintf)
