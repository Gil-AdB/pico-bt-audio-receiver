# CMakeLists.txt for Pico BT Audio Receiver
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2_w)
set(PICO_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk)
include(pico_sdk_import.cmake)

project(pico_bt_audio C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${PROJECT_NAME})

target_sources(${PROJECT_NAME} PRIVATE
    src/main.c
    src/bt_audio_sink.c
    src/i2s_audio.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_cyw43_arch_threadsafe_background
    pico_btstack_classic
    pico_btstack_cyw43
    pico_btstack_sbc_decoder
    hardware_pio
    hardware_dma
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    PICO_CYW43_ARCH_BLUETOOTH_ENABLED=1
    CYW43_LWIP=0
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_add_extra_outputs(${PROJECT_NAME})
