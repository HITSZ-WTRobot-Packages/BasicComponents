add_library(LibsPrintf STATIC
        "printf.c"
)

target_include_directories(LibsPrintf
        PUBLIC
        .
)

# link dependencies if any


# alias for external use
add_library(libs::printf ALIAS LibsPrintf)
