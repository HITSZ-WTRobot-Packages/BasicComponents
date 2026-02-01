add_library(___utils INTERFACE)

target_include_directories(___utils INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(___utils INTERFACE stm32cubemx)

# alias for external use
add_library(utils ALIAS ___utils)
