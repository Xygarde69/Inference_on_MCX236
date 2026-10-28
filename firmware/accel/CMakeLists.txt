# Accelerometer readout for FRDM-MCXN236 (freestanding MCUXpresso SDK project).
# Build from the repo root:  uv run west build -p always accel -b frdmmcxn236 -d build

cmake_minimum_required(VERSION 3.22.0)

include(${SdkRootDirPath}/cmake/extension/mcux.cmake)

project(accel LANGUAGES C CXX ASM)

include(${SdkRootDirPath}/CMakeLists.txt)

mcux_add_source(
    SOURCES main.c
            board/hardware_init.c
            board/app.h
            board/pin_mux.c
            board/pin_mux.h
)

mcux_add_include(
    INCLUDES .
             board
)

# SDK_I2C_BASED_COMPONENT_USED enables BOARD_Accel_I2C_* in the SDK's board.c
mcux_add_macro(
    CC "-DSDK_I2C_BASED_COMPONENT_USED=1\
       -DPRINTF_ADVANCED_ENABLE=1"
)

mcux_convert_binary(BINARY ${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin)
