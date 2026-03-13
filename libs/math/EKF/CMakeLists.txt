add_library(__Math_EKF INTERFACE)

target_include_directories(__Math_EKF INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(__Math_EKF INTERFACE Math::LinearAlgebra)

# alias for external use
add_library(Math::EKF ALIAS __Math_EKF)
