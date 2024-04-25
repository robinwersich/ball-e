# This must be included before calling pico_sdk_init()

if (DEFINED ENV{BLUEPAD32_ROOT} AND (NOT BLUEPAD32_ROOT))
    set(BLUEPAD32_ROOT $ENV{BLUEPAD32_ROOT})
    message("Using BLUEPAD32_ROOT from environment ('${BLUEPAD32_ROOT}')")
endif ()

set(BTSTACK_ROOT ${BLUEPAD32_ROOT}/external/btstack)
set(PICO_BTSTACK_PATH ${BTSTACK_ROOT})
