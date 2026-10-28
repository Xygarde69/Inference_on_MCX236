# OnePolicy inference + tilt command for FRDM-MCXN236 (freestanding MCUXpresso SDK project).
# The weights come from ../policy (policy/export.py writes policy_model.[ch] from the ONNX).
# Build from firmware/:  uv run west build -p always app -b frdmmcxn236 -d build --config release

cmake_minimum_required(VERSION 3.22.0)

include(${SdkRootDirPath}/cmake/extension/mcux.cmake)

project(policy LANGUAGES C CXX ASM)

include(${SdkRootDirPath}/CMakeLists.txt)

mcux_add_source(
    SOURCES main.c
            policy_net.c
            policy_net.h
            tilt.c
            tilt.h
            ../policy/policy_model.c
            ../policy/policy_model.h
            board/hardware_init.c
            board/app.h
            board/pin_mux.c
            board/pin_mux.h
)

mcux_add_include(
    INCLUDES .
             ../policy
             board
)

# SDK_I2C_BASED_COMPONENT_USED enables BOARD_Accel_I2C_* in the SDK's board.c
mcux_add_macro(
    CC "-DSDK_I2C_BASED_COMPONENT_USED=1"
)

# The MLP is the hot loop: optimize it fully, even in debug builds
set_source_files_properties(policy_net.c PROPERTIES COMPILE_OPTIONS "-O3")

mcux_convert_binary(BINARY ${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin)
