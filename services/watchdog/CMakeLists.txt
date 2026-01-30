add_library(ServicesWatchdog STATIC
    "./watchdog.cpp"
)

target_include_directories(ServicesWatchdog
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any


# alias for external use
add_library(services::Watchdog ALIAS ServicesWatchdog)
