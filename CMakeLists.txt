cmake_minimum_required(VERSION 3.25)
project(tgxc C)

find_package(SDL2 REQUIRED)

include_directories(tgxc PUBLIC ${SDL2_INCLUDE_DIRS} include)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(tgxc src/main.c
        include/TGX.h
        src/TGX.c
        include/graphics_cache_map.h)

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

target_link_directories(tgxc PUBLIC ${SDL2_LIBRARIES})
target_link_libraries(tgxc PUBLIC ${SDL2_LIBRARIES} m)

add_subdirectory(tgas)