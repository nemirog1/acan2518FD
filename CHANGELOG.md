# Changelog

## 1.0.0

First release of the fork. Base: **ACAN2517FD 2.1.16** by Pierre Molinaro (MIT).

### Added

- Sleep and bus wake-up support, the reason this fork exists. Upstream's `begin()`
  enables RXIE, TXIE and TXATIE and never sets **WAKIE**, so a controller slept
  through the public API wakes on bus activity without ever asserting its INT pin.
  New public methods:
  - `maskInterruptsForSleep()` — WAKIE on, everything else off, flags cleared,
    previous enables saved.
  - `restoreInterruptsAfterSleep()` — put them back.
  - `setWakeUpFilter(enabled, wft)` — `CiCON.WAKFIL`/`WFT`, verified by read-back.
  - `setOperationModeAndWait(mode, ms)` — bounded mode request.
  - `enterSleepMode(ms)` — Sleep, verified by `OPMOD` **or** `OSCDIS`.
  - `isAsleep()` — `OSC.OSCDIS`.
  - `wakeUpFromSleepMode(ms)` — bounded oscillator restart, usable **before**
    `begin()`, which matters because a host MCU reset does not reset the
    controller.
- `examples/SleepWakeDemoESP32` — the full cycle on an ESP32. Not yet verified on
  hardware.
- An explicit clamp to the MCP2518FD's 20 MHz SPI ceiling (DS20006027). No
  behavioural change at any supported oscillator.

### Changed

- Renamed `ACAN2517FD` → `ACAN2518FD` throughout: class, settings, filters, file
  names, examples. Shared ACAN types (`CANFDMessage`, `CANMessage`,
  `ACANFDBuffer`) deliberately unchanged.
- Scope narrowed to the MCP2518FD. Upstream also serves the MCP2517FD and the
  MCP251863.
- `library.properties`, `keywords.txt` and `README.md` rewritten for the fork.
- `LICENSE` retains Pierre Molinaro's copyright and adds one for the fork's
  changes.

### Unchanged

Everything else, including the bit-timing solver, the filter API, the message
types, and the behaviour of every existing method.
