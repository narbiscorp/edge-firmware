# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# FCC test build switch. The separate narbiscorp/edge-firmware-fcc CI sets
# FCC_TEST_BUILD=1 in the environment to compile the DTM (Direct Test Mode)
# path in main.c. Unset — the normal case, including this repo's own CI — is a
# production build with that code excluded entirely by the preprocessor, so the
# shipped binary is byte-identical to one built before the FCC work landed.
# The project name stays ESP32_Ble either way: the OTA updater validates the
# bin's embedded project_name against that prefix.
if(DEFINED ENV{FCC_TEST_BUILD} AND NOT "$ENV{FCC_TEST_BUILD}" STREQUAL "0")
    add_compile_definitions(FCC_TEST_BUILD=1)
    message(STATUS "*** FCC TEST BUILD — DTM test mode COMPILED IN (not for release) ***")
endif()

project(ESP32_Ble)
