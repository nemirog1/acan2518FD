# ACAN2518FD

An Arduino driver for the **MCP2518FD** CAN controller in CAN FD mode, with
**sleep and bus wake-up support**.

> **This is a fork.** It is derived from
> [ACAN2517FD](https://github.com/pierremolinaro/acan2517FD) **2.1.16** by
> **Pierre Molinaro**, MIT licensed. Pierre wrote the driver, the bit-timing
> solver and everything that makes it good; this fork narrows it to one part and
> adds one missing capability. If you need the MCP2517FD or the MCP251863, or you
> do not need sleep, **use his library** — it is the better-maintained choice.

---

## Why this fork exists

`ACAN2517FD::begin()` configures the controller's interrupts like this
(ACAN2517FD.cpp, lines 451-456 in 2.1.16):

```cpp
data8  = (1 << 1) ;  // Receive FIFO Interrupt Enable
data8 |= (1 << 0) ;  // Transmit FIFO Interrupt Enable
writeRegister8 (INT_REGISTER + 2, data8) ;
data8  = (1 << 2) ;  // TXATIE
writeRegister8 (INT_REGISTER + 3, data8) ;
```

`CiINT` bit 30 — **WAKIE** — is never set.

So a controller put into Sleep mode through the public API *does* wake on a
dominant edge at RXCAN — that transition is automatic in silicon — but it
**asserts nothing on its INT pin**. A host MCU watching that pin sleeps straight
through the event. Any design that arms a CAN INT line as a wake source silently
does nothing at all.

There was no way to fix that from outside the library: `writeRegister8()` and its
siblings are `private`, and there is no wake-related call in the public API.

---

## What changed against upstream 2.1.16

Nothing else differs. Normalise the rename away and a `diff` produces exactly this
list.

### 1. Renamed

`ACAN2517FD` → `ACAN2518FD` throughout — class, settings, filters, file names.
Pierre's authorship headers and the upstream URLs are kept in every file.

The shared ACAN types — `CANFDMessage`, `CANMessage`, `ACANFDBuffer` — are
**unchanged**, so code written against any ACAN driver still compiles. That also
means this library and ACAN2517FD **cannot be used in the same sketch**: both
define those types.

### 2. Specialised for the MCP2518FD

Upstream drives the MCP2517FD, the MCP2518FD and the MCP251863 from one code base.
In practice there is almost nothing chip-specific in it to remove — same register
map, same oscillator options, same GPIO handling, same RAM size — so this is
mostly a narrowing of scope in the documentation rather than of the code.

### 3. SPI clock ceiling stated explicitly

```cpp
const uint32_t MCP2518FD_MAX_SPI_CLOCK = 20UL * 1000UL * 1000UL ; // DS20006027
```

Upstream computes `0.4 × SYSCLK` and applies it unclamped. At every oscillator the
part supports that is already under the ceiling, so this changes no behaviour. It
is here so the device limit lives in the code rather than emerging from an
arithmetic expression.

### 4. The sleep/wake API

Seven public methods and three private members. Nothing existing was modified.

| Method | What it does |
|---|---|
| `maskInterruptsForSleep()` | Sets **WAKIE**, clears RXIE/TXIE/TXATIE, clears pending flags, saves the previous enables. |
| `restoreInterruptsAfterSleep()` | Puts the saved enables back. No-op if never masked. |
| `setWakeUpFilter(enabled, wft)` | `CiCON.WAKFIL` / `WFT`, **read back and verified**. Must be called in Configuration mode. |
| `setOperationModeAndWait(mode, ms)` | Request a mode and poll `CiCON.OPMOD` until it matches, bounded. |
| `enterSleepMode(ms)` | Request Sleep and verify. |
| `isAsleep()` | `OSC.OSCDIS` — the oscillator is stopped, i.e. the part is asleep. |
| `wakeUpFromSleepMode(ms)` | Clear `OSCDIS`, wait **bounded** for `OSCRDY`. **Safe to call before `begin()`.** |

---

## Using it

### Going to sleep

```cpp
//--- 1. Drain anything queued. A frame left in the RX FIFO holds RXIF, and RXIF
//       holds the INT pin low - which would wake the host the instant it slept.
CANFDMessage frame ;
while (can.receive (frame)) {}

//--- 2. Only a wake-up may assert INT from here on.
can.maskInterruptsForSleep () ;

//--- 3. Configuration mode FIRST. It is always reachable, never waits on bus
//       idle, and is the only mode in which the CiCON config bits are writable.
can.setOperationModeAndWait (ACAN2518FDSettings::Configuration, 20) ;

//--- 4. Optional: reject glitches on a parked bus.
can.setWakeUpFilter (true, 0) ;

//--- 5. Park the transceiver, if your board can (e.g. through the controller's
//       GPIO0 pin wired to the PHY standby input). Its low-power wake receiver is
//       then what watches the bus. Board-specific - see gpioWrite()/gpioSetMode().

//--- 6. Sleep, verified.
if (!can.enterSleepMode (20)) {
  // Refused. Stay awake - an awake controller works; a half-slept one does not.
}
```

Then arm your MCU to wake on the INT pin going **low**, and sleep it.

### Coming back

After a host **deep sleep** the MCU resets but **the controller does not** — there
is no reset line between them. Every controller is still asleep with its
oscillator stopped, and `begin()` opens by requesting a mode change and issuing a
chip RESET against exactly that stopped oscillator. So:

```cpp
SPI.begin (...) ;
can.wakeUpFromSleepMode () ;   // BEFORE begin(). Harmless if already awake.
can.begin (settings, ...) ;
```

After a host **light sleep**, resume in place instead:

```cpp
can.wakeUpFromSleepMode () ;
// ... un-park the transceiver ...
can.setOperationModeAndWait (yourNormalMode, 20) ;
can.restoreInterruptsAfterSleep () ;
```

`examples/SleepWakeDemoESP32` puts the whole cycle together.

### Two design notes

**`enterSleepMode()` accepts either witness** — `OPMOD == Sleep`, or `OSCDIS`
set. In Sleep the oscillator is stopped, and how OPMOD reads back through a
stopped clock is not worth staking a power design on. A false negative here means
never sleeping at all, which is worse than a slightly loose check.

**`wakeUpFromSleepMode()` is not a duplicate of
`performSleepModeToConfigurationMode()`.** That one's readiness loop is unbounded,
so an absent or dead controller wedges the caller forever, and it is not usable
before `begin()` — which is exactly when it is needed.

---

## Everything else

Identical to ACAN2517FD. Bit-rate configuration, the timing solver, reception
filters, the `CANFDMessage` API, interrupt and polled operation, the `examples/`
sketches — all Pierre's, all unchanged in behaviour.

- **Full driver API documentation:** the PDF in `extras/` (written for
  ACAN2517FD; the API is the same, with the type renamed).
- **Upstream repository:** https://github.com/pierremolinaro/acan2517FD

### Quick start

```cpp
#include <ACAN2518FD.h>
#include <SPI.h>

static const byte MCP2518_CS  = 20 ;   // adapt to your design
static const byte MCP2518_INT = 37 ;

ACAN2518FD can (MCP2518_CS, SPI, MCP2518_INT) ;

void setup () {
  Serial.begin (9600) ;
  while (!Serial) {}
  SPI.begin () ;
  ACAN2518FDSettings settings (ACAN2518FDSettings::OSC_4MHz10xPLL,
                               125 * 1000, ACAN2518FDSettings::DATA_BITRATE_x4) ;
  const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  if (errorCode != 0) {
    Serial.print ("Error 0x") ; Serial.println (errorCode, HEX) ;
  }
}
```

---

## Migrating from ACAN2517FD

Rename three types and the include. Nothing else changes.

| Was | Now |
|---|---|
| `#include <ACAN2517FD.h>` | `#include <ACAN2518FD.h>` |
| `ACAN2517FD` | `ACAN2518FD` |
| `ACAN2517FDSettings` | `ACAN2518FDSettings` |
| `ACAN2517FDFilters` | `ACAN2518FDFilters` |

Remember that both libraries define `CANFDMessage` and `CANMessage`, so a sketch
can use one or the other, never both.

---

## Status

**The sleep/wake path has not yet been confirmed on hardware.** It is derived from
the MCP2518FD datasheet and reviewed carefully, and everything here compiles, but
until it has been measured on a board treat it as a considered proposal rather
than a proven one. Reports either way are welcome.

The rest of the library is upstream 2.1.16, which has been in the field for years.

---

## Credit and licence

MIT, and the copyright is shared: **Pierre Molinaro** for the original work and
everything that still is his, and the fork's own changes on top. See `LICENSE`.

If the sleep/wake support is useful to you, it would be better in Pierre's library
than in a fork — say so upstream. If it lands there, this repository can retire.
