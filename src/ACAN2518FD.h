//------------------------------------------------------------------------------
// ACAN2518FD - a CAN driver for the MCP2518FD, CAN FD mode
//
// Fork of ACAN2517FD 2.1.16 by Pierre Molinaro
//     https://github.com/pierremolinaro/acan2517FD        (MIT - see LICENSE)
// This fork
//     https://github.com/nemirog1/acan2518FD
//
// Upstream drives the MCP2517FD, the MCP2518FD and the MCP251863 from one code
// base. This fork targets the MCP2518FD only, and adds the sleep/wake support
// upstream does not have: a controller slept through the upstream API wakes on
// bus activity but asserts nothing on its INT pin, because WAKIE is never
// enabled. README.md has the evidence and the full list of changes.
//
// Register comments cite DS20005688B, the MCP2517FD datasheet, with its page
// numbers. That is Pierre's original referencing and the register map is shared
// between the parts, so the citations remain correct. The MCP2518FD's own
// datasheet is DS20006027.
//------------------------------------------------------------------------------

#pragma once

//------------------------------------------------------------------------------

#include <ACAN2518FDSettings.h>
#include <ACAN2518FD_ACANFDBuffer.h>
#include <ACAN2518FD_CANMessage.h>
#include <ACAN2518FDFilters.h>
#include <SPI.h>

//------------------------------------------------------------------------------
//   ACAN2518FD class
//------------------------------------------------------------------------------

class ACAN2518FD {

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //   CONSTRUCTOR
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: ACAN2518FD (const uint8_t inCS, // CS input of MCP2518FD
                      SPIClass & inSPI, // Hardware SPI object
                      const uint8_t inINT) ; // INT output of MCP2518FD

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //   begin method (returns 0 if no error)
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: uint32_t begin (const ACAN2518FDSettings & inSettings,
                          void (* inInterruptServiceRoutine) (void)) ;

  public: uint32_t begin (const ACAN2518FDSettings & inSettings,
                          void (* inInterruptServiceRoutine) (void),
                          const ACAN2518FDFilters & inFilters) ;

//--- Error code returned by begin
  public: static const uint32_t kRequestedConfigurationModeTimeOut  = uint32_t (1) <<  0 ;
  public: static const uint32_t kReadBackErrorWith1MHzSPIClock      = uint32_t (1) <<  1 ;
  public: static const uint32_t kTooFarFromDesiredBitRate           = uint32_t (1) <<  2 ;
  public: static const uint32_t kInconsistentBitRateSettings        = uint32_t (1) <<  3 ;
  public: static const uint32_t kINTPinIsNotAnInterrupt             = uint32_t (1) <<  4 ;
  public: static const uint32_t kISRIsNull                          = uint32_t (1) <<  5 ;
  public: static const uint32_t kFilterDefinitionError              = uint32_t (1) <<  6 ;
  public: static const uint32_t kMoreThan32Filters                  = uint32_t (1) <<  7 ;
  public: static const uint32_t kControllerReceiveFIFOSizeIsZero    = uint32_t (1) <<  8 ;
  public: static const uint32_t kControllerReceiveFIFOSizeGreaterThan32 = uint32_t (1) << 9 ;
  public: static const uint32_t kControllerTransmitFIFOSizeIsZero    = uint32_t (1) << 10 ;
  public: static const uint32_t kControllerTransmitFIFOSizeGreaterThan32 = uint32_t (1) << 11 ;
  public: static const uint32_t kControllerRamUsageGreaterThan2048   = uint32_t (1) << 12 ;
  public: static const uint32_t kControllerTXQPriorityGreaterThan31  = uint32_t (1) << 13 ;
  public: static const uint32_t kControllerTransmitFIFOPriorityGreaterThan31 = uint32_t (1) << 14 ;
  public: static const uint32_t kControllerTXQSizeGreaterThan32     = uint32_t (1) << 15 ;
  public: static const uint32_t kRequestedModeTimeOut               = uint32_t (1) << 16 ;
  public: static const uint32_t kX10PLLNotReadyWithin1MS            = uint32_t (1) << 17 ;
  public: static const uint32_t kReadBackErrorWithFullSpeedSPIClock = uint32_t (1) << 18 ;
  public: static const uint32_t kISRNotNullAndNoIntPin              = uint32_t (1) << 19 ;
  public: static const uint32_t kInvalidTDCO                        = uint32_t (1) << 20 ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //   end method (resets the MCP2518FD, deallocate buffers, and detach interrupt pin)
  //   Return true if end method succeeds, and false otherwise
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: bool end (void) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //   Send a message
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: bool tryToSend (const CANFDMessage & inMessage) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Receive a message
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: bool receive (CANFDMessage & outMessage) ;
  public: bool available (void) ;
  public: typedef void (*tFilterMatchCallBack) (const uint32_t inFilterIndex) ;
  public: bool dispatchReceivedMessage (const tFilterMatchCallBack inFilterMatchCallBack = NULL) ;

//--- Call back function array
  private: ACANFDCallBackRoutine * mCallBackFunctionArray = NULL ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Get error counters
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: uint32_t errorCounters (void) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Get diagnostic information (thanks to Flole998 and turmary)
  // inIndex == 0 returns BDIAG0_REGISTER
  // inIndex != 0 returns BDIAG1_REGISTER
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: uint32_t diagInfos (const int inIndex = 1) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Operation Mode
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: ACAN2518FDSettings::OperationMode currentOperationMode (void) ;

  public: void setOperationMode (const ACAN2518FDSettings::OperationMode inMode) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Recovery from Restricted Operation Mode
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: bool recoverFromRestrictedOperationMode (void) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Sleep Mode to Configuration Mode
  // (returns true if MCP2518FD was in sleep mode)
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: bool performSleepModeToConfigurationMode (void) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Sleep / wake  (ADDED IN THIS FORK - see README.md)
  //
  //  The intended order for a system sleep is:
  //
  //      maskInterruptsForSleep () ;                      // WAKIE only
  //      setOperationModeAndWait (Configuration, 20) ;    // always reachable
  //      setWakeUpFilter (true, 0) ;                      // needs Configuration
  //      ... park the transceiver ...
  //      enterSleepMode (20) ;                            // verified
  //
  //  and to come back in place (light sleep, or an aborted attempt):
  //
  //      wakeUpFromSleepMode (5) ;                        // oscillator first
  //      ... un-park the transceiver ...
  //      setOperationModeAndWait (yourNormalMode, 20) ;
  //      restoreInterruptsAfterSleep () ;
  //
  //  After a DEEP sleep on the host MCU none of this is needed on the way back:
  //  that is a host reset and begin() re-initialises the controller - EXCEPT that
  //  the controller itself was NOT reset (no reset line), so wakeUpFromSleepMode()
  //  must still run before begin(), which is why it works before begin() ever has.
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  /// Request an operating mode and poll CiCON.OPMOD until it matches. Returns
  /// false on timeout rather than blocking forever.
  public: bool setOperationModeAndWait (const ACAN2518FDSettings::OperationMode inMode,
                                        const uint32_t inTimeoutMs = 20) ;

  /// Mask every interrupt source except the wake-up, and clear the pending flags.
  ///
  /// THIS IS THE POINT OF THE FORK. begin() enables RXIE, TXIE and TXATIE and
  /// never touches WAKIE, so a controller slept as the library leaves it wakes on
  /// bus activity - that transition is automatic in silicon - but asserts NOTHING
  /// on its INT pin, and a host watching that pin sleeps straight through it.
  /// Clearing the other three matters just as much in the other direction: an INT
  /// held low by an ordinary RX or TX condition when the host arms a low-level
  /// wake is an instant wake, and then a sleep/wake loop.
  ///
  /// The previous enable bytes are saved for restoreInterruptsAfterSleep().
  public: void maskInterruptsForSleep (void) ;

  /// Put back the interrupt enables saved by maskInterruptsForSleep() and clear
  /// the pending flags. No-op if the mask was never applied.
  public: void restoreInterruptsAfterSleep (void) ;

  /// Configure the wake-up filter (CiCON.WAKFIL / CiCON.WFT), which rejects
  /// glitches on RXCAN so noise on a parked bus does not rouse the system.
  /// inFilterTime is WFT, 0..3, longer being more immune and less sensitive.
  ///
  /// MUST be called in Configuration mode - several CiCON bits are writable only
  /// there. The write is READ BACK, and the return value says what the device
  /// actually has, so a filter that failed to arm is a fact rather than an
  /// assumption.
  public: bool setWakeUpFilter (const bool inEnabled, const uint8_t inFilterTime = 0) ;

  /// True when the controller's oscillator is disabled, which is the state Sleep
  /// mode leaves it in. This is the independent witness that the part is actually
  /// asleep: it is the same bit performSleepModeToConfigurationMode() uses.
  public: bool isAsleep (void) ;

  /// Request Sleep and verify. Accepts EITHER witness - OPMOD reading Sleep, or
  /// OSCDIS set - because reading OPMOD back through a stopped oscillator is not
  /// something to stake a power design on, and a false negative here means never
  /// sleeping at all. Returns false only if neither appears within the timeout.
  public: bool enterSleepMode (const uint32_t inTimeoutMs = 20) ;

  /// Clear OSCDIS and wait, BOUNDED, for the oscillator to report ready.
  ///
  /// Two differences from performSleepModeToConfigurationMode(), which does
  /// roughly the same job: this one cannot hang (that one's readiness loop has no
  /// timeout, so an absent or dead controller wedges the caller forever), and this
  /// one is safe to call BEFORE begin() - it brings up CS itself and uses a
  /// conservative 800 kHz for the transfers, restoring the previous SPI settings
  /// afterwards. That matters because a host MCU reset does not reset this chip:
  /// after a deep sleep every controller is still asleep with its oscillator
  /// stopped, and begin() opens by resetting against exactly that.
  public: bool wakeUpFromSleepMode (const uint32_t inTimeoutMs = 5) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Private properties
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #ifdef ARDUINO_ARCH_ESP32
    private: TaskHandle_t mESP32TaskHandle = nullptr ;
  #endif
  private: SPISettings mSPISettings ;
  private: SPIClass & mSPI ;
  private: const uint8_t mCS ;
  private: const uint8_t mINT ;
  private: bool mUsesTXQ ;
  private: bool mHardwareTxFIFOFull ;
  private: bool mRxInterruptEnabled ; // Added in 2.1.7
  private: uint8_t mTransmitFIFOPayload ; // in byte count
  private: uint8_t mTXQBufferPayload ; // in byte count
  private: uint8_t mReceiveFIFOPayload ; // in byte count
  private: uint8_t mTXBWS_RequestedMode ;
  private: uint8_t mHardwareReceiveBufferOverflowCount ;
//--- Sleep/wake state (ADDED IN THIS FORK)
  private: uint8_t mSavedInterruptEnable2 = 0 ; // CiINT+2 before the sleep mask
  private: uint8_t mSavedInterruptEnable3 = 0 ; // CiINT+3 before the sleep mask
  private: bool mInterruptsMaskedForSleep = false ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Receive buffer
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  private: ACANFDBuffer mDriverReceiveBuffer ;

  public: uint32_t driverReceiveBufferPeakCount (void) const {
    return mDriverReceiveBuffer.peakCount () ;
  }

  public: uint8_t hardwareReceiveBufferOverflowCount (void) const {
    return mHardwareReceiveBufferOverflowCount ;
  }

  public: void resetHardwareReceiveBufferOverflowCount (void) {
    mHardwareReceiveBufferOverflowCount = 0 ;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Transmit buffer
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  private: ACANFDBuffer mDriverTransmitBuffer ;

  public: uint32_t driverTransmitBufferSize (void) const {
    return mDriverTransmitBuffer.size () ;
  }

  public: uint32_t driverTransmitBufferCount (void) const {
    return mDriverTransmitBuffer.count () ;
  }

  public: uint32_t driverTransmitBufferPeakCount (void) const {
    return mDriverTransmitBuffer.peakCount () ;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Private methods
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  private: void writeRegister32Assume_SPI_transaction (const uint16_t inRegisterAddress, const uint32_t inValue) ;
  private: void writeRegister8Assume_SPI_transaction (const uint16_t inRegisterAddress, const uint8_t inValue) ;

  private: uint32_t readRegister32Assume_SPI_transaction (const uint16_t inRegisterAddress) ;
  private: uint8_t readRegister8Assume_SPI_transaction (const uint16_t inRegisterAddress) ;
  private: uint16_t readRegister16Assume_SPI_transaction (const uint16_t inRegisterAddress) ;

  private: void reset2518FD (void) ;

  private: void writeRegister8 (const uint16_t inRegisterAddress, const uint8_t inValue) ;
  private: void writeRegister32 (const uint16_t inAddress, const uint32_t inValue) ;

  private: uint8_t readRegister8 (const uint16_t inAddress) ;
  private: uint16_t readRegister16 (const uint16_t inAddress) ;
  private: uint32_t readRegister32 (const uint16_t inAddress) ;

  private: bool sendViaTXQ (const CANFDMessage & inMessage) ;
  private: bool enterInTransmitBuffer (const CANFDMessage & inMessage) ;
  private: void appendInControllerTxFIFO (const CANFDMessage & inMessage) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Polling
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: void poll (void) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Interrupt service routine
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: void isr (void) ;
  public: void isr_poll_core (void) ;
  private: void receiveInterrupt (void) ;
  private: void transmitInterrupt (void) ;
  #ifdef ARDUINO_ARCH_ESP32
    public: SemaphoreHandle_t mISRSemaphore ;
  #endif

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Optimized CS handling (thanks to Flole998)
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #if defined(__AVR__)
    private: volatile uint8_t *cs_pin_reg;
    private: uint8_t cs_pin_mask;
    private: inline void initCS () {
      cs_pin_reg = portOutputRegister(digitalPinToPort(mCS));
      cs_pin_mask = digitalPinToBitMask(mCS);
      pinMode(mCS, OUTPUT);
    }
    private: inline void assertCS() {
      *(cs_pin_reg) &= ~cs_pin_mask;
    }
    private: inline void deassertCS() {
      *(cs_pin_reg) |= cs_pin_mask;
    }
  #elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK66FX1M0__) || defined(__MK64FX512__)
    private: volatile uint8_t *cs_pin_reg;
    private: inline void initCS () {
      cs_pin_reg = portOutputRegister(mCS);
      pinMode(mCS, OUTPUT);
    }
    private: inline void assertCS() {
      *(cs_pin_reg+256) = 1;
    }
    private: inline void deassertCS() {
      *(cs_pin_reg+128) = 1;
    }
  #elif defined(__MKL26Z64__)
    private: volatile uint8_t *cs_pin_reg;
    private: uint8_t cs_pin_mask;
    private: inline void initCS () {
      cs_pin_reg = portOutputRegister(digitalPinToPort(mCS));
      cs_pin_mask = digitalPinToBitMask(mCS);
      pinMode(mCS, OUTPUT);
    }
    private: inline void assertCS() {
      *(cs_pin_reg+8) = cs_pin_mask;
    }
    private: inline void deassertCS() {
      *(cs_pin_reg+4) = cs_pin_mask;
    }
  #elif defined(__SAM3X8E__) || defined(__SAM3A8C__) || defined(__SAM3A4C__)
    private: volatile uint32_t *cs_pin_reg;
    private: uint32_t cs_pin_mask;
    private: inline void initCS () {
      cs_pin_reg = &(digitalPinToPort(mCS)->PIO_PER);
      cs_pin_mask = digitalPinToBitMask(mCS);
      pinMode(mCS, OUTPUT);
    }
    private: inline void assertCS() {
      *(cs_pin_reg+13) = cs_pin_mask;
    }
    private: inline void deassertCS() {
      *(cs_pin_reg+12) = cs_pin_mask;
    }
  #elif defined(__PIC32MX__)
    private: volatile uint32_t *cs_pin_reg;
    private: uint32_t cs_pin_mask;
    private: inline void initCS () {
      cs_pin_reg = portModeRegister(digitalPinToPort(mCS));
      cs_pin_mask = digitalPinToBitMask(mCS);
      pinMode(mCS, OUTPUT);
    }
    private: inline void assertCS() {
      *(cs_pin_reg+8+1) = cs_pin_mask;
    }
    private: inline void deassertCS() {
      *(cs_pin_reg+8+2) = cs_pin_mask;
    }
  #elif defined(ARDUINO_ARCH_ESP8266)
    private: uint32_t cs_pin_mask;
    private: inline void initCS () {
      cs_pin_mask = 1 << mCS;
      pinMode(mCS, OUTPUT);
    }
    private: inline void assertCS() {
      GPOC = cs_pin_mask;
    }
    private: inline void deassertCS() {
      GPOS = cs_pin_mask;
    }

  #elif defined(__SAMD21G18A__)
    private: volatile uint32_t *cs_pin_reg;
    private: uint32_t cs_pin_mask;
    private: inline void initCS () {
      cs_pin_reg = portModeRegister(digitalPinToPort(mCS));
      cs_pin_mask = digitalPinToBitMask(mCS);
      pinMode(mCS, OUTPUT);
    }
    private: inline void assertCS() {
      *(cs_pin_reg+5) = cs_pin_mask;
    }
    private: inline void deassertCS() {
      *(cs_pin_reg+6) = cs_pin_mask;
    }
  #else
    private: inline void initCS () {
      pinMode (mCS, OUTPUT);
    }
    private: inline void assertCS() {
      digitalWrite(mCS, LOW);
    }
    private: inline void deassertCS() {
      digitalWrite(mCS, HIGH);
    }
  #endif

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    GPIO
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: void gpioSetMode (const uint8_t inPin, const uint8_t inMode) ;

  public: void gpioWrite (const uint8_t inPin, const uint8_t inLevel) ;

  public: bool gpioRead (const uint8_t inPin) ;

  public: void configureGPIO0AsXSTBY (void) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    No copy
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  private: ACAN2518FD (const ACAN2518FD &) = delete ;
  private: ACAN2518FD & operator = (const ACAN2518FD &) = delete ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

} ;

//------------------------------------------------------------------------------

