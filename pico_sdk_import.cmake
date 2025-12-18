# pico_sdk_import.cmake
set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../pico-sdk" CACHE PATH "Path to the Raspberry Pi Pico SDK")

if (NOT EXISTS ${PICO_SDK_PATH})
    message(FATAL_ERROR "Pico SDK not found at ${PICO_SDK_PATH}. Please update the path.")
endif()

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)