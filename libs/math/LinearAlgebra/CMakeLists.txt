add_library(__Math_LinearAlgebra INTERFACE)

target_include_directories(__Math_LinearAlgebra INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any


# alias for external use
add_library(Math::LinearAlgebra ALIAS __Math_LinearAlgebra)
