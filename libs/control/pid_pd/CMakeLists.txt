add_library(LibsPIDPD STATIC
        "pid_pd.cpp"
)

target_include_directories(LibsPIDPD
        PUBLIC
        .
)

# link dependencies if any


# alias for external use
add_library(libs::PID_PD ALIAS LibsPIDPD)
