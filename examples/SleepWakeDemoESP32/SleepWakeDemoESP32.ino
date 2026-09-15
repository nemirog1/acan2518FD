//——————————————————————————————————————————————————————————————————————————————
//  ACAN2518FD sleep / wake demo, for ESP32
//
//  Shows the whole cycle the fork exists for:
//
//    quiet bus  ->  park the controller in Sleep  ->  deep-sleep the ESP32
//                                                  -> traffic on the bus
//                                                  -> INT goes low
//                                                  -> ESP32 wakes and re-inits
//
//  WHAT YOU NEED
//    * an MCP2518FD on SPI, with its INT pin on an RTC-CAPABLE GPIO - the ESP32
//      EXT1 wake source only works on those. On the original ESP32 those are
//      GPIO 0,2,4,12-15,25-27,32-39; on the ESP32-S3, GPIO 0-21. GPIO32 below is
//      RTC-capable on the original ESP32.
//    * a real CAN bus with a transceiver. This demo CANNOT run in loopback: a
//      loopback controller never sees a dominant edge on RXCAN, so it never
//      wakes.
//
//  NOT YET VERIFIED ON HARDWARE. The sequence follows the MCP2518FD datasheet
//  and the ESP32 sleep documentation, but at the time of writing nobody has
//  measured it. Treat it as a starting point, not as proof.
//——————————————————————————————————————————————————————————————————————————————

#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board"
#endif

//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2518FD.h>
#include <SPI.h>
#include "esp_sleep.h"
#include "esp_idf_version.h"

//——————————————————————————————————————————————————————————————————————————————
//   WIRING - adapt to your board
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2518_SCK  = 26 ; // SCK input of MCP2518FD
static const byte MCP2518_MOSI = 19 ; // SDI input of MCP2518FD
static const byte MCP2518_MISO = 18 ; // SDO output of MCP2518FD

static const byte MCP2518_CS  = 16 ; // CS input of MCP2518FD
static const byte MCP2518_INT = 32 ; // INT output of MCP2518FD - MUST be RTC-capable

//--- Sleep after this much silence on the bus
static const uint32_t IDLE_BEFORE_SLEEP_MS = 10UL * 1000UL ;

//——————————————————————————————————————————————————————————————————————————————
//   Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2518FD can (MCP2518_CS, SPI, MCP2518_INT) ;

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup () {
  Serial.begin (115200) ;
  delay (200) ;   // no `while (!Serial)`: after a deep-sleep wake nobody is watching

//--- Why are we running?
  switch (esp_sleep_get_wakeup_cause ()) {
  case ESP_SLEEP_WAKEUP_EXT1 :
    Serial.println ("Woke on CAN bus activity") ;
    break ;
  case ESP_SLEEP_WAKEUP_UNDEFINED :
    Serial.println ("Cold boot") ;
    break ;
  default :
    Serial.println ("Woke from some other source") ;
    break ;
  }

//--- Begin SPI
  SPI.begin (MCP2518_SCK, MCP2518_MISO, MCP2518_MOSI) ;

//--- THE STEP THAT IS EASY TO MISS.
//    The ESP32 reset that ends a deep sleep does NOT reset the MCP2518FD - there
//    is no reset line between them. The controller is therefore still in Sleep
//    mode with its oscillator stopped, and begin() opens by requesting a mode
//    change and issuing a chip RESET against exactly that stopped oscillator.
//    Restart the oscillator first. Harmless on a cold boot - the part is already
//    awake, and nothing is written.
  if (!can.wakeUpFromSleepMode ()) {
    Serial.println ("Controller oscillator did not start - check wiring/power") ;
  }

//--- Ordinary bring-up. 500 kbit/s arbitration, classic-compatible.
  ACAN2518FDSettings settings (ACAN2518FDSettings::OSC_40MHz, 500UL * 1000UL,
                               ACAN2518FDSettings::DATA_BITRATE_x1) ;
  settings.mRequestedMode = ACAN2518FDSettings::Normal20B ;
  const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  if (errorCode == 0) {
    Serial.println ("Controller up") ;
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————
//   GOING TO SLEEP
//
//   The order is the design. Anything that can refuse, refuses BEFORE the board
//   has given up the ability to say so.
//——————————————————————————————————————————————————————————————————————————————

static void goToSleep () {
  Serial.println ("Bus quiet - going to sleep") ;
  Serial.flush () ;

//--- 1. Drain. A frame left in the RX FIFO holds RXIF, and RXIF holds INT LOW -
//       which would wake the ESP32 the instant it slept, forever.
  CANFDMessage frame ;
  while (can.receive (frame)) {}

//--- 2. Mask every interrupt but the wake-up. Without this the controller wakes
//       on bus activity and tells nobody: upstream ACAN2517FD never enables
//       WAKIE. Clearing the others matters just as much in the other direction -
//       an INT held low by an ordinary TX condition is an instant wake.
  can.maskInterruptsForSleep () ;

//--- 3. Configuration mode FIRST. It is always reachable, it never waits on bus
//       idle, and it is the only mode in which the CiCON config bits are
//       writable. Asking for Sleep straight from Normal means asking for a mode
//       change while the PHY may already be holding RXCAN dominant.
  if (!can.setOperationModeAndWait (ACAN2518FDSettings::Configuration, 20)) {
    Serial.println ("Could not reach Configuration mode - staying awake") ;
    can.restoreInterruptsAfterSleep () ;
    return ;
  }

//--- 4. Reject glitches on a parked bus. Verified by read-back.
  const bool filterOn = can.setWakeUpFilter (true, 0) ;
  if (!filterOn) {
    Serial.println ("Wake-up filter did not arm - any dominant edge will wake us") ;
  }

//--- 5. Park the transceiver here, if your board routes the PHY's standby pin to
//       the controller's GPIO0. Its low-power wake receiver is then what watches
//       the bus. Board-specific; with GPIO0 wired straight to an active-HIGH
//       standby input it is:
//           can.gpioWrite (0, HIGH) ; can.gpioSetMode (0, OUTPUT) ;

//--- 6. Sleep, verified. A silent failure here is the worst outcome available:
//       the ESP32 asleep while the controller stays awake, drawing full current
//       and unable to wake anybody.
  if (!can.enterSleepMode (20)) {
    Serial.println ("Controller would not enter Sleep - staying awake") ;
    can.setOperationModeAndWait (ACAN2518FDSettings::Normal20B, 20) ;
    can.restoreInterruptsAfterSleep () ;
    return ;
  }

//--- 7. INT must be HIGH before we arm a LOW-level wake, or we wake instantly.
  pinMode (MCP2518_INT, INPUT_PULLUP) ;
  const uint32_t deadline = millis () + 250 ;
  while (digitalRead (MCP2518_INT) == LOW) {
    if (millis () > deadline) {
      Serial.println ("INT still asserted - staying awake") ;
      can.wakeUpFromSleepMode () ;
      can.setOperationModeAndWait (ACAN2518FDSettings::Normal20B, 20) ;
      can.restoreInterruptsAfterSleep () ;
      return ;
    }
  }

//--- 8. Arm EXT1 on the INT pin and stop.
  const uint64_t mask = 1ULL << MCP2518_INT ;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  esp_sleep_enable_ext1_wakeup_io (mask, ESP_EXT1_WAKEUP_ANY_LOW) ;
#else
  esp_sleep_enable_ext1_wakeup (mask, ESP_EXT1_WAKEUP_ANY_LOW) ;
#endif
  Serial.println ("Sleeping. Put a frame on the bus to wake me.") ;
  Serial.flush () ;
  esp_deep_sleep_start () ;   // does not return; next thing that runs is setup()
}

//——————————————————————————————————————————————————————————————————————————————
//   LOOP
//——————————————————————————————————————————————————————————————————————————————

static uint32_t gLastTrafficMs = 0 ;

void loop () {
  CANFDMessage frame ;
  if (can.receive (frame)) {
    gLastTrafficMs = millis () ;
    Serial.print ("Received 0x") ;
    Serial.println (frame.id, HEX) ;
  }
  if ((millis () - gLastTrafficMs) > IDLE_BEFORE_SLEEP_MS) {
    goToSleep () ;
    gLastTrafficMs = millis () ;   // only reached if the sleep was refused
  }
}

//——————————————————————————————————————————————————————————————————————————————
