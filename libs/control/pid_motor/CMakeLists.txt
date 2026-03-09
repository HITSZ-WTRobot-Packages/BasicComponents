add_library(LibsPIDMotor STATIC
        "pid_motor.cpp"
)

target_include_directories(LibsPIDMotor
        PUBLIC
        .
)

# link dependencies if any


# alias for external use
add_library(libs::PIDMotor ALIAS LibsPIDMotor)
