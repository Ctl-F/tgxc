project(tgas CXX)

include_directories(tgas PUBLIC include ../include)

add_executable(tgas src/main.cpp)

if(CMAKE_BUILD_TYPE MATCHES DEBUG)

    message("Debug Configuration")
    add_compile_definitions(_DEBUG_)

elseif(CMAKE_BUILD_TYPE MATCHES RELEASE)

    message("Release Configuration")
    add_compile_definitions(_DIST_)
    add_compile_definitions(_RELEASE_)

else()

    message("Release With Debug Configuration")
    add_compile_definitions(_RELEASE_)

endif()

