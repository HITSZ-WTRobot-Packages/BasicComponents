add_library(LibsMITPD STATIC
    "./mit_pd.cpp"
)

target_include_directories(LibsMITPD
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any


# alias for external use
add_library(libs::MIT_PD ALIAS LibsMITPD)
