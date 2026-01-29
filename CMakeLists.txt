cmake_minimum_required(VERSION 3.10)
project(motor_api C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Include directories
include_directories(
    include
    src
    /usr/local/include
)

# Source files
set(SOURCES
    src/core/ma_core.c
    src/core/ma_cycle.c
    src/drivers/drv_cia402.c
    src/drivers/drv_hans.c
    src/drivers/drv_io.c
    src/utils/ma_utils.c
    src/utils/ma_config.c
    src/motor_api_facade.c
    src/motor_api_net.c
    src/cJSON.c
)

# Shared Library
add_library(motor_api SHARED ${SOURCES})
target_link_libraries(motor_api ethercat pthread rt m)

# Static Library
add_library(motor_api_static STATIC ${SOURCES})
target_link_libraries(motor_api_static ethercat pthread rt m)

# Examples
add_executable(example_csp examples/example_csp.c)
target_link_libraries(example_csp motor_api)

add_executable(test_io examples/test_io.c)
target_link_libraries(test_io motor_api)

add_executable(test_full_raw examples/test_full_raw.c)
target_link_libraries(test_full_raw ethercat pthread rt)

add_executable(test_interactive examples/test_interactive.c)
target_link_libraries(test_interactive motor_api)

# Tests (Optional, keep existing tests)
# add_executable(test_read test/test_read.c)
# target_link_libraries(test_read motor_api)

# Install
install(TARGETS motor_api motor_api_static
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(FILES include/motor_api.h include/motor_api_types.h include/motor_api_common.h include/motor_api_net.h
    DESTINATION include
)
