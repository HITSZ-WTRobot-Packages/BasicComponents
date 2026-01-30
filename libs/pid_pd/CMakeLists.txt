add_library(LibsPIDPD STATIC
    "./pid_pd.cpp"
)

target_include_directories(LibsPIDPD
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any


# alias for external use
add_library(libs::PID_PD ALIAS LibsPIDPD)
