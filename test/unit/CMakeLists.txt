cmake_minimum_required(VERSION 3.16)

# ESP-IDF Unity test component.
# Run with: idf.py -C test/unit build flash monitor
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(test_door_fsm)

idf_component_register(
    SRCS
        "test_door_fsm.c"
        "../../src/fsm/door_fsm.c"
        "../../src/nvs/fault_nvs.c"
    INCLUDE_DIRS
        "../../include"
        "../../src"
    REQUIRES
        unity
        freertos
        nvs_flash
        esp_timer
        esp_system
)

# Expose internal FSM functions for white-box testing.
target_compile_definitions(${COMPONENT_LIB} PRIVATE FSM_UNIT_TEST)