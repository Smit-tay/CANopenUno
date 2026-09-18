cmake_minimum_required(VERSION 3.27)
project(arduino_canopen C CXX)

include(FetchContent)

# 1. Fetch Cory Fowler's MCP2515 CAN Library
FetchContent_Declare(
    mcp_can_lib
    GIT_REPOSITORY https://github.com/coryjfowler/MCP_CAN_lib.git
    GIT_TAG        34807287506afc6b4aef7c577392c1c4e32d1b45  # master as of 2026-09-10
)

# 2. Fetch the Official Arduino AVR Core Framework
FetchContent_Declare(
    arduino_core
    GIT_REPOSITORY https://github.com/arduino/ArduinoCore-avr.git
    GIT_TAG        11b9130371e8447920edb65a75706a6c951e51fc  # master as of 2026-09-10
)

FetchContent_MakeAvailable(mcp_can_lib arduino_core)

# 3. Explicitly calculate include paths right now so we can force them into the compiler flags
set(ARDUINO_CORE_DIR "${arduino_core_SOURCE_DIR}/cores/arduino")
set(ARDUINO_VARIANT_DIR "${arduino_core_SOURCE_DIR}/variants/standard")
set(ARDUINO_SPI_DIR "${arduino_core_SOURCE_DIR}/libraries/SPI/src")

# Collate the raw source implementation files
file(GLOB ARDUINO_CORE_SRCS 
    "${ARDUINO_CORE_DIR}/*.c" 
    "${ARDUINO_CORE_DIR}/*.cpp"
    "${ARDUINO_SPI_DIR}/*.cpp"
)

# Strip out standard main.cpp so it doesn't conflict with your custom loop code
list(REMOVE_ITEM ARDUINO_CORE_SRCS "${ARDUINO_CORE_DIR}/main.cpp")

# 4. Compile everything directly in a single executable target (Bypasses avr-ar entirely!)
add_executable(${PROJECT_NAME}.elf 
    src/main.cpp
    ${mcp_can_lib_SOURCE_DIR}/mcp_can.cpp
    ${ARDUINO_CORE_SRCS}
)

# Push the include paths directly into the global compile flags for the executable
target_include_directories(${PROJECT_NAME}.elf PRIVATE 
    "${PROJECT_SOURCE_DIR}/src"
    "${mcp_can_lib_SOURCE_DIR}"
    "${ARDUINO_CORE_DIR}"
    "${ARDUINO_VARIANT_DIR}"
    "${ARDUINO_SPI_DIR}"
)

# Convert the raw compiled binary target into a lightweight flashing hex image
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Building Standalone Flash Image Asset: ${PROJECT_NAME}.hex"
)

# GUI Target mapping for flashing directly over COM4
add_custom_target(flash
    COMMAND ${AVRDUDE_EXECUTABLE} -F -V -c arduino -p m328p -P COM4 -b 115200 -U flash:w:${CMAKE_BINARY_DIR}/arduino_canopen.hex:i
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Flashing firmware to physical Arduino Uno on COM4..."
)