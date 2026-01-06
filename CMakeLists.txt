cmake_minimum_required(VERSION 3.15)
project(motor_api C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(MOTOR_API_WERROR "Treat warnings as errors" OFF)

add_compile_options(-Wall -Wextra -O2)
if(MOTOR_API_WERROR)
  add_compile_options(-Werror)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(motor_api_static STATIC
  src/motor_api.c
  src/motor_api_common.c
  src/motor_api_net.c
  src/motor_api_config.c
  src/cJSON.c
)
target_link_libraries(motor_api_static ethercat pthread m)

add_library(motor_api_shared SHARED
  src/motor_api.c
  src/motor_api_common.c
  src/motor_api_net.c
  src/motor_api_config.c
  src/cJSON.c
)
target_link_libraries(motor_api_shared ethercat pthread m)
set_target_properties(motor_api_shared PROPERTIES OUTPUT_NAME motor_api)

add_executable(example_csp examples/example_csp.c)
target_link_libraries(example_csp motor_api_static ethercat pthread)

add_executable(test_read examples/test_read.c)
target_link_libraries(test_read motor_api_static ethercat pthread)

add_executable(test_eth_initdll examples/test_eth_initdll.c)
target_link_libraries(test_eth_initdll motor_api_static ethercat pthread)

add_executable(test_main test/test_main.c)
target_link_libraries(test_main motor_api_static ethercat pthread m)

add_executable(test_config test/test_config.c)
target_link_libraries(test_config motor_api_static ethercat pthread m)

add_executable(test_config_dual test/test_config_dual.c)
target_link_libraries(test_config_dual motor_api_static ethercat pthread m)

add_executable(test_angle_loop test/test_angle_loop.c)
target_link_libraries(test_angle_loop motor_api_static ethercat pthread m)

add_executable(test_clear_faults_all test/test_clear_faults_all.c)
target_link_libraries(test_clear_faults_all motor_api_static ethercat pthread m)

install(TARGETS motor_api_static motor_api_shared example_csp test_read test_eth_initdll
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(FILES include/motor_api.h include/motor_api_common.h include/motor_api_net.h include/motor_api_types.h DESTINATION include)
