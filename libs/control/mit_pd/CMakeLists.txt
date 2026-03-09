add_library(LibsMITPD STATIC
        "mit_pd.cpp"
)

target_include_directories(LibsMITPD
        PUBLIC
        .
)

# link dependencies if any


# alias for external use
add_library(libs::MIT_PD ALIAS LibsMITPD)
