# Third-party licenses

This project's own code (see `LICENSE`) is licensed under MIT. At build time, `CMakeLists.txt`
fetches and statically links two third-party libraries into the final firmware image. Neither is
vendored in this repository — both are pulled via CMake `FetchContent` from their own upstream repos
at the pinned commits given in `CMakeLists.txt`, so this repo's own source is sufficient to reproduce
the exact combination that ends up on the device.

## MCP_CAN_lib

- **Author:** Cory Fowler ([coryjfowler](https://github.com/coryjfowler))
- **Repository:** https://github.com/coryjfowler/MCP_CAN_lib
- **License:** GNU Lesser General Public License v3.0 (LGPL-3.0)
- Originally forked from [Longan-Labs/Arduino_CAN_BUS_MCP2515](https://github.com/Longan-Labs/Arduino_CAN_BUS_MCP2515).

Provides the MCP2515/MCP25625 CAN controller driver this firmware uses for all CAN bus I/O.

## ArduinoCore-avr

- **Author:** Arduino SA
- **Repository:** https://github.com/arduino/ArduinoCore-avr
- **License:** GNU Lesser General Public License v2.1 (or later)

Provides the ATmega328P core runtime (startup code, `SPI` library, standard Arduino API) this
firmware builds against.

## LGPL and static linking on AVR

Both dependencies above are LGPL, which was written assuming dynamic linking, where a user can
relink against a modified copy of the library. That's not possible on an AVR target — this firmware
is one statically-linked `.elf`. What LGPL requires in that case is that the pieces needed to relink
(i.e. rebuild) with a modified version of either library be available to anyone receiving the work.
That's satisfied here: `CMakeLists.txt` fetches both libraries' full source directly from their
official repositories, so cloning this repo and pointing `FetchContent` at a modified fork of either
dependency is exactly the recompilation path LGPL is protecting.
