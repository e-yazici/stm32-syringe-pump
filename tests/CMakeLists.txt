cmake_minimum_required(VERSION 3.20)
project(SyringePump_UnitTests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

# Compiler warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -pedantic -Wno-unused-parameter)
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/unity
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/Libs/Inc
)

# Unity testing library
add_library(unity STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/unity/unity.c
)

# Common portable domain logic library
add_library(syringe_pump_domain STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/Libs/Src/pump_kinematics.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/Libs/Src/stepper_profile.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/Libs/Src/syringe_db.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/Libs/Src/as5600_math.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/Libs/Src/ui_editor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/Libs/Src/printf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/Libs/Src/closed_loop.c
)
if(NOT MSVC)
    target_link_libraries(syringe_pump_domain PUBLIC m)
endif()

# Function to register a test executable
function(add_syringe_test TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME} PRIVATE unity syringe_pump_domain)
    if(NOT MSVC)
        target_link_libraries(${TEST_NAME} PRIVATE m)
    endif()
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

# Core driver and domain unit test suites (1-to-1 with Drivers/Libs modules)
add_syringe_test(test_pump_kinematics ${CMAKE_CURRENT_SOURCE_DIR}/test_pump_kinematics.c)
add_syringe_test(test_stepper_profile ${CMAKE_CURRENT_SOURCE_DIR}/test_stepper_profile.c)
add_syringe_test(test_syringe_db ${CMAKE_CURRENT_SOURCE_DIR}/test_syringe_db.c)
add_syringe_test(test_as5600_math ${CMAKE_CURRENT_SOURCE_DIR}/test_as5600_math.c)
add_syringe_test(test_ui_editor ${CMAKE_CURRENT_SOURCE_DIR}/test_ui_editor.c)
add_syringe_test(test_ui_menu ${CMAKE_CURRENT_SOURCE_DIR}/test_ui_menu.c)
add_syringe_test(test_closed_loop ${CMAKE_CURRENT_SOURCE_DIR}/test_closed_loop.c)

add_executable(test_tmc2208
    ${CMAKE_CURRENT_SOURCE_DIR}/test_tmc2208.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/Libs/Src/TMC2208.c
)
target_link_libraries(test_tmc2208 PRIVATE unity syringe_pump_domain)
if(NOT MSVC)
    target_link_libraries(test_tmc2208 PRIVATE m)
endif()
add_test(NAME test_tmc2208 COMMAND test_tmc2208)
