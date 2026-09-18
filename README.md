# CANopenUno

A minimal CANopen slave node running on an Arduino Uno, used as real test hardware (node 32) in an industrial automation platform.

This isn't a general-purpose CANopen library. It's a small, purpose-built slave whose only job is to
give the master something real to talk to: NMT state transitions, SDO expedited/segmented read and
write, SYNC-triggered TPDO, and RPDO reception, all exercised against real CAN hardware rather than
a simulator.

## Hardware

- **MCU:** Arduino Uno (ATmega328P)
- **CAN controller:** standalone Microchip MCP2515 CAN controller + TJA1050 high-speed CAN transceiver
  breakout module
- **Crystal:** 8 MHz (`MCP_8MHZ`)
- **CAN bus bit rate:** 250 kbps (`CAN_250KBPS`) — deliberately matched to the other test platform (Jhoinrch dongle) so this node interoperates on the same shared bus

### Wiring

Hardware SPI (D10-D13) plus a dedicated interrupt line on D2.

| MCP2515 pin | Arduino Uno pin | Notes |
|---|---|---|
| VCC | 5V | Powers the TJA1050 transceiver |
| GND | GND | Shared ground reference |
| CS | Pin 10 | Hardware Slave Select |
| SO (MISO) | Pin 12 | Master In, Slave Out |
| SI (MOSI) | Pin 11 | Master Out, Slave In |
| SCK | Pin 13 | Hardware SPI clock |
| INT | Pin 2 | Interrupt on incoming CAN frame |

Node-id: **32**, assigned via `NODE_ID` in the firmware (not LSS-assigned — this node boots directly
into its configured id).

## What it does

- **NMT:** responds to master-issued Start / Stop / Enter-Pre-Operational / Reset-Node /
  Reset-Communication, following the standard CiA301 default (boots into Pre-Operational, waits for
  an explicit Start before producing PDOs).
- **SDO server:** expedited and segmented upload/download, both directions, including boundary sizes
  and standard aborts (`NOT_EXIST`, `READONLY`, `DATA_LONG`). Only answers requests addressed to its
  own node-id (`rxId` checked against `0x600 + NODE_ID`) — an earlier version answered SDO traffic
  meant for other nodes on the bus, found and fixed while testing the master's multi-channel SDO
  client.
- **TPDO1:** SYNC-triggered (switched over from a free-running timer). The old free-running timer is
  kept alongside at an offset period purely for visual comparison on a trace/scope, not as a fallback.
- **RPDO:** receives and applies incoming process data from the master.
- **Object dictionary:** a small, purpose-built OD table — not a generated one — covering the objects
  this test node actually needs plus a spread of standard CiA301 datatypes (0x2000-0x200B range) used
  to exercise the master's SDO client against every basic type, including a VISIBLE_STRING object used
  to prove out segmented string transfer.

## Architecture

Firmware is organized around a single message-class dispatch table (`classify()` routes each incoming
frame to one handler per class: NMT / SYNC / EMCY / TIME / PDO / SDO / heartbeat / other) — the same
taxonomy the master driver itself uses for trace filtering, so behavior on both ends of the bus is
easy to reason about side by side.

## Toolchain

Built with AVR-GCC and flashed with AVRDUDE (both vendored outside the corporate build system). No Arduino IDE / `arduino-cli` dependency.

## License

This project's own code is MIT licensed (see `LICENSE`). It statically links two third-party
libraries, fetched at build time rather than vendored — [MCP_CAN_lib](https://github.com/coryjfowler/MCP_CAN_lib)
(Cory Fowler, LGPL-3.0) and [ArduinoCore-avr](https://github.com/arduino/ArduinoCore-avr)
(Arduino SA, LGPL-2.1-or-later). See `THIRD_PARTY_LICENSES.md` for full attribution.

## Status

Verified on real hardware against the master: NMT state transitions, SDO in both
directions (including concurrent multi-channel transfers from the master), SYNC-triggered TPDO, and
RPDO reception all confirmed working together with the Windows and Raspberry Pi test nodes on one
shared CAN bus.
