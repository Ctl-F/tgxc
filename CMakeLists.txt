cmake_minimum_required(VERSION 3.25)
project(TGXC C)

find_package(SDL2 REQUIRED)

include_directories(TGXC PUBLIC ${SDL2_INCLUDE_DIRS} include)

set(CMAKE_C_STANDARD 11)

add_executable(TGXC src/main.c
        include/TGX.h
        src/TGX.c)

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

target_link_directories(TGXC PUBLIC ${SDL2_LIBRARIES})
target_link_libraries(TGXC PUBLIC ${SDL2_LIBRARIES} m)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -march=native")
endif()