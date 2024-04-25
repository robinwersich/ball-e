add_library(btcontrol STATIC
    ./src/btcontrol.cpp
)

set(BTCONTROL_INCLUDE_DIRS
    # Needed for btstack_config.h / sdkconfig.h
    # so that libbluepad32 can include them
    ./config
    # Needed when using BTstack from Bluepad32 branch
    ${BTSTACK_ROOT}/3rd-party/bluedroid/encoder/include
    ${BTSTACK_ROOT}/3rd-party/bluedroid/decoder/include
)

target_include_directories(btcontrol PUBLIC ${BTCONTROL_INCLUDE_DIRS})

target_include_directories(btcontrol PUBLIC
    ./include
    ${BTCONTROL_INCLUDE_DIRS}
)

target_link_libraries(btcontrol PUBLIC
    pico_stdlib
    pico_cyw43_arch_none
    pico_btstack_classic
    pico_btstack_cyw43
    bluepad32
)

include_directories(${BTCONTROL_INCLUDE_DIRS})

add_subdirectory(${BLUEPAD32_ROOT}/src/components/bluepad32 libbluepad32)
