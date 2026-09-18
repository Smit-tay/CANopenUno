# ==============================================================================
# INDEPENDENT AVR ATMEGA328P CROSS-COMPILATION TOOLCHAIN PROFILE
# Isolated completely from the corporate build system
# ==============================================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)

# Point straight to your verified integration sysroot binaries
set(AVR_BIN_ROOT "C:/devtools/avr-gcc-15.1.0/avr-gcc-15.1.0-x64-windows/bin")

# Define cross-compilation binaries
set(CMAKE_C_COMPILER   "${AVR_BIN_ROOT}/avr-gcc.exe")
set(CMAKE_CXX_COMPILER "${AVR_BIN_ROOT}/avr-g++.exe")
set(CMAKE_OBJCOPY      "${AVR_BIN_ROOT}/avr-objcopy.exe")
set(AVRDUDE_EXECUTABLE "${AVR_BIN_ROOT}/avrdude.exe")

# Bypass host platform verification tests
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

# Enforce strict 8-bit AVR micro-code memory layout optimization models
set(ARDUINO_MCU "atmega328p" CACHE STRING "Target hardware MCU processor architecture")
set(ARDUINO_F_CPU "16000000L" CACHE STRING "Target hardware clock speed designation")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

add_compile_options(
    -mmcu=${ARDUINO_MCU}
    -DF_CPU=${ARDUINO_F_CPU}
    -Os -w -ffunction-sections -fdata-sections -flto
)
add_link_options(-mmcu=${ARDUINO_MCU} -Os -Wl,--gc-sections -flto)