add_library(__Math_Geometry INTERFACE)

target_include_directories(__Math_Geometry INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(__Math_Geometry INTERFACE Math::LinearAlgebra)

# alias for external use
add_library(Math::Geometry ALIAS __Math_Geometry)
