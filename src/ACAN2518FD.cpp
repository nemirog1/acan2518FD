//------------------------------------------------------------------------------
// A CAN driver for MCP2518FD, CANFD mode
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2517FD
//
// Forked for the MCP2518FD from ACAN2517FD 2.1.16 (MIT, Pierre Molinaro) - see README.md
//------------------------------------------------------------------------------

#include <ACAN2518FD.h>

//------------------------------------------------------------------------------

static const uint8_t TXBWS = 0 ;

//------------------------------------------------------------------------------
// Note about ESP32
//------------------------------------------------------------------------------
//
// It appears that Arduino ESP32 interrupts are managed in a completely different way from "usual" Arduino:
//   - SPI.usingInterrupt is not implemented;
//   - noInterrupts() and interrupts() are NOPs: use taskDISABLE_INTERRUPTS and taskENABLE_INTERRUPTS;
//   - interrupt service routines should be fast, otherwise you get an "Guru Meditation Error: Core 1 panic'ed
//     (Interrupt wdt timeout on CPU1)".
//
// So we handle the ESP32 interrupt in the following way:
//   - interrupt service routine performs a xSemaphoreGiveFromISR on mISRSemaphore of can driver
//   - this activates the myESP32Task task that performs "isr_poll_core" that is done by interrupt service routine
//     in "usual" Arduino;
//   - as this task runs in parallel with setup / loop routines, SPI access is natively protected by the
//     beginTransaction / endTransaction pair, that manages a mutex;
//   - (May 29, 2019) it appears that MCP2518FD wants the CS line to deasserted as soon as possible (thanks for
//     Nick Kirkby for having signaled me this point, see https://github.com/pierremolinaro/acan2517/issues/5);
//     so we mask interrupts when we access the MCP2518FD, the sequence becomes:
//           mSPI.beginTransaction (mSPISettings) ;
//             #ifdef ARDUINO_ARCH_ESP32
//               taskDISABLE_INTERRUPTS () ;
//             #endif
//               assertCS () ;
//                  ... Access the MCP2518FD ...
//               deassertCS () ;
//             #ifdef ARDUINO_ARCH_ESP32
//               taskENABLE_INTERRUPTS () ;
//             #endif
//           mSPI.endTransaction () ;
//
//------------------------------------------------------------------------------

#ifdef ARDUINO_ARCH_ESP32
  static void myESP32Task (void * pData) {
    ACAN2518FD * canDriver = (ACAN2518FD *) pData ;
    while (1) {
      xSemaphoreTake (canDriver->mISRSemaphore, portMAX_DELAY) ;
      canDriver->isr_poll_core () ;
    }
  }
#endif

//------------------------------------------------------------------------------
// ACAN2518FD register addresses
//------------------------------------------------------------------------------

static const uint16_t CON_REGISTER      = 0x000 ;
static const uint16_t NBTCFG_REGISTER   = 0x004 ;
static const uint16_t DBTCFG_REGISTER   = 0x008 ;
static const uint16_t TDC_REGISTER      = 0x00C ;

static const uint16_t TREC_REGISTER     = 0x034 ;
static const uint16_t BDIAG0_REGISTER   = 0x038 ;
static const uint16_t BDIAG1_REGISTER   = 0x03C ;

//------------------------------------------------------------------------------
//   TXQ REGISTERS
//------------------------------------------------------------------------------

static const uint16_t TXQCON_REGISTER   = 0x050 ;
static const uint16_t TXQSTA_REGISTER   = 0x054 ;
static const uint16_t TXQUA_REGISTER    = 0x058 ;

//------------------------------------------------------------------------------
//   INTERRUPT REGISTERS
//------------------------------------------------------------------------------

static const uint16_t INT_REGISTER = 0x01C ;

//------------------------------------------------------------------------------
//   FIFO REGISTERS
//------------------------------------------------------------------------------

static uint16_t FIFOCON_REGISTER (const uint16_t inFIFOIndex) { // 1 ... 31
  return 0x05C + 12 * (inFIFOIndex - 1) ;
}

//------------------------------------------------------------------------------

static uint16_t FIFOSTA_REGISTER (const uint16_t inFIFOIndex) { // 1 ... 31
  return 0x060 + 12 * (inFIFOIndex - 1) ;
}

//------------------------------------------------------------------------------

static uint16_t FIFOUA_REGISTER (const uint16_t inFIFOIndex) { // 1 ... 31
  return 0x064 + 12 * (inFIFOIndex - 1) ;
}

//------------------------------------------------------------------------------
//   FILTER REGISTERS
//------------------------------------------------------------------------------

static uint16_t FLTCON_REGISTER (const uint16_t inFilterIndex) { // 0 ... 31 (DS20005688B, page 58)
  return 0x1D0 + inFilterIndex ;
}

//------------------------------------------------------------------------------

static uint16_t FLTOBJ_REGISTER (const uint16_t inFilterIndex) { // 0 ... 31 (DS20005688B, page 60)
  return 0x1F0 + 8 * inFilterIndex ;
}

//------------------------------------------------------------------------------

static uint16_t MASK_REGISTER (const uint16_t inFilterIndex) { // 0 ... 31 (DS20005688B, page 61)
  return 0x1F4 + 8 * inFilterIndex ;
}

//------------------------------------------------------------------------------
//   OSCILLATOR REGISTER
//------------------------------------------------------------------------------

static const uint16_t OSC_REGISTER = 0xE00 ;

//------------------------------------------------------------------------------
//   INPUT / OUPUT CONTROL REGISTER
//------------------------------------------------------------------------------

static const uint16_t IOCON_REGISTER_00_07 = 0xE04 ;
static const uint16_t IOCON_REGISTER_08_15 = 0xE05 ;
static const uint16_t IOCON_REGISTER_16_23 = 0xE06 ;
static const uint16_t IOCON_REGISTER_24_31 = 0xE07 ;

//------------------------------------------------------------------------------
//    RECEIVE FIFO INDEX
//------------------------------------------------------------------------------

static const uint8_t RECEIVE_FIFO_INDEX  = 1 ;
static const uint8_t TRANSMIT_FIFO_INDEX = 2 ;

//------------------------------------------------------------------------------
//    BYTE BUFFER UTILITY FUNCTIONS
//------------------------------------------------------------------------------

static void enterU32InBufferAtIndex (const uint32_t inValue, uint8_t ioBuffer [], const uint8_t inIndex) {
  ioBuffer [inIndex + 0] = (uint8_t) inValue ;
  ioBuffer [inIndex + 1] = (uint8_t) (inValue >>  8) ;
  ioBuffer [inIndex + 2] = (uint8_t) (inValue >> 16) ;
  ioBuffer [inIndex + 3] = (uint8_t) (inValue >> 24) ;
}

//------------------------------------------------------------------------------

static uint32_t u32FromBufferAtIndex (uint8_t ioBuffer [], const uint8_t inIndex) {
  uint32_t result = (uint32_t) ioBuffer [inIndex + 0] ;
  result |= ((uint32_t) ioBuffer [inIndex + 1]) <<  8 ;
  result |= ((uint32_t) ioBuffer [inIndex + 2]) << 16 ;
  result |= ((uint32_t) ioBuffer [inIndex + 3]) << 24 ;
  return result ;
}

//------------------------------------------------------------------------------

static uint16_t u16FromBufferAtIndex (uint8_t ioBuffer [], const uint8_t inIndex) {
  uint16_t result = (uint16_t) ioBuffer [inIndex + 0] ;
  result |= ((uint16_t) ioBuffer [inIndex + 1]) <<  8 ;
  return result ;
}

//------------------------------------------------------------------------------

ACAN2518FD::ACAN2518FD (const uint8_t inCS, // CS input of MCP2518FD
                        SPIClass & inSPI, // Hardware SPI object
                        const uint8_t inINT) : // INT output of MCP2518FD
mSPISettings (),
mSPI (inSPI),
mCS (inCS),
mINT (inINT),
mUsesTXQ (false),
mHardwareTxFIFOFull (false),
mRxInterruptEnabled (true),
mTransmitFIFOPayload (0),
mTXQBufferPayload (0),
mReceiveFIFOPayload (0),
mTXBWS_RequestedMode (0),
mHardwareReceiveBufferOverflowCount (0),
mDriverReceiveBuffer (),
mDriverTransmitBuffer ()
#ifdef ARDUINO_ARCH_ESP32
  , mISRSemaphore (xSemaphoreCreateCounting (10, 0))
#endif
{
}

//------------------------------------------------------------------------------

uint32_t ACAN2518FD::begin (const ACAN2518FDSettings & inSettings,
                            void (* inInterruptServiceRoutine) (void)) {
//--- Add pass-all filter
  ACAN2518FDFilters filters ;
  filters.appendPassAllFilter (NULL) ;
//---
  return begin (inSettings, inInterruptServiceRoutine, filters) ;
}

//------------------------------------------------------------------------------

uint32_t ACAN2518FD::begin (const ACAN2518FDSettings & inSettings,
                            void (* inInterruptServiceRoutine) (void),
                            const ACAN2518FDFilters & inFilters) {
  uint32_t errorCode = 0 ; // Means no error
//----------------------------------- If ok, check if settings are correct
  if (!inSettings.mArbitrationBitRateClosedToDesiredRate) {
    errorCode |= kTooFarFromDesiredBitRate ;
  }
  if (inSettings.CANBitSettingConsistency () != 0) {
    errorCode |= kInconsistentBitRateSettings ;
  }
//----------------------------------- Check mINT has interrupt capability
  const int8_t itPin = digitalPinToInterrupt (mINT) ;
  if ((mINT != 255) && (itPin == NOT_AN_INTERRUPT)) {
    errorCode = kINTPinIsNotAnInterrupt ;
  }
//----------------------------------- Check interrupt service routine is not null
  if ((mINT != 255) && (inInterruptServiceRoutine == NULL)) {
    errorCode |= kISRIsNull ;
  }
//----------------------------------- Check consistency between ISR and INT pin
  if ((mINT == 255) && (inInterruptServiceRoutine != NULL)) {
    errorCode |= kISRNotNullAndNoIntPin ;
  }
//----------------------------------- Check TXQ size is <= 32
  if (inSettings.mControllerTXQSize > 32) {
    errorCode |= kControllerTXQSizeGreaterThan32 ;
  }
//----------------------------------- Check TXQ priority is <= 31
  if (inSettings.mControllerTXQBufferPriority > 31) {
    errorCode |= kControllerTXQPriorityGreaterThan31 ;
  }
//----------------------------------- Check controller receive FIFO size is 1 ... 32
  if (inSettings.mControllerReceiveFIFOSize == 0) {
    errorCode |= kControllerReceiveFIFOSizeIsZero ;
  }else if (inSettings.mControllerReceiveFIFOSize > 32) {
    errorCode |= kControllerReceiveFIFOSizeGreaterThan32 ;
  }
//----------------------------------- Check controller transmit FIFO size is 1 ... 32
  if (inSettings.mControllerTransmitFIFOSize == 0) {
    errorCode |= kControllerTransmitFIFOSizeIsZero ;
  }else if (inSettings.mControllerTransmitFIFOSize > 32) {
    errorCode |= kControllerTransmitFIFOSizeGreaterThan32 ;
  }
//----------------------------------- Check Transmit FIFO priority is <= 31
  if (inSettings.mControllerTransmitFIFOPriority > 31) {
    errorCode |= kControllerTransmitFIFOPriorityGreaterThan31 ;
  }
//----------------------------------- Check MCP2518FD controller RAM usage is <= 2048 bytes
  if (inSettings.ramUsage () > 2048) {
    errorCode |= kControllerRamUsageGreaterThan2048 ;
  }
//----------------------------------- Check Filter definition
  if (inFilters.filterCount () > 32) {
    errorCode |= kMoreThan32Filters ;
  }
  if (inFilters.filterStatus () != ACAN2518FDFilters::kFiltersOk) {
    errorCode |= kFilterDefinitionError ;
  }
//----------------------------------- Check TDCO value
  if ((inSettings.mTDCO > 63) || (inSettings.mTDCO < -64)) {
    errorCode |= kInvalidTDCO ;
  }
//----------------------------------- INT, CS pins, reset MCP2518FD
  if (errorCode == 0) {
    if (mINT != 255) { // 255 means interrupt is not used (thanks to Tyler Lewis)
      pinMode (mINT, INPUT_PULLUP) ;
    }
    initCS () ;
  //----------------------------------- Set SPI clock to 800 kHz
    mSPISettings = SPISettings (800UL * 1000, MSBFIRST, SPI_MODE0) ;
  //----------------------------------- Request configuration mode
    bool wait = true ;
    const uint32_t startTime = millis () ;
    while (wait) {
      writeRegister8 (CON_REGISTER + 3, 0x04 | (1 << 3)) ; // Request configuration mode, abort all transmissions
      const uint8_t actualMode = (readRegister8 (CON_REGISTER + 2) >> 5) & 0x07 ;
      wait = actualMode != 0x04 ;
      if (wait && ((millis () - startTime) > 2)) { // Wait (2 ms max) until the configuration mode is reached
        errorCode |= kRequestedConfigurationModeTimeOut ;
        wait = false ;
      }
    }
  //----------------------------------- Reset MCP2518FD (always use a 800 kHz clock)
    reset2518FD () ;
  }
//----------------------------------- Check SPI connection is on (with a 800 kHz clock)
// We write and the read back MCP2518FD RAM at address 0x400
  for (uint32_t i=1 ; (i != 0) && (errorCode == 0) ; i <<= 1) {
    const uint16_t RAM_WORD_ADDRESS = 0x400 ;
    writeRegister32 (RAM_WORD_ADDRESS, i) ;
    const uint32_t readBackValue = readRegister32 (RAM_WORD_ADDRESS) ;
    if (readBackValue != i) {
      errorCode = kReadBackErrorWith1MHzSPIClock ;
    }
  }
//----------------------------------- Now, set internal clock with OSC register
//     Bit 0: (rw) 1 --> 10xPLL
//     Bit 4: (rw) 0 --> SCLK is divided by 1, 1 --> SCLK is divided by 2
//     Bits 5-6: Clock Output Divisor
  if (errorCode == 0) {
    uint8_t pll = 0 ; // No PLL
    uint8_t osc = 0 ; // Divide by 1
    switch (inSettings.oscillator ()) {
    case ACAN2518FDSettings::OSC_4MHz:
    case ACAN2518FDSettings::OSC_20MHz:
    case ACAN2518FDSettings::OSC_40MHz:
      break ;
    case ACAN2518FDSettings::OSC_4MHz_DIVIDED_BY_2:
    case ACAN2518FDSettings::OSC_20MHz_DIVIDED_BY_2:
    case ACAN2518FDSettings::OSC_40MHz_DIVIDED_BY_2:
      osc = 1 << 4 ; // Divide by 2
      break ;
    case ACAN2518FDSettings::OSC_4MHz10xPLL_DIVIDED_BY_2 :
      pll = 1 ; // Enable 10x PLL
      osc = 1 << 4 ; // Divide by 2
      break ;
    case ACAN2518FDSettings::OSC_4MHz10xPLL :
      pll = 1 ; // Enable 10x PLL
      break ;
    }
    osc |= pll ;
    if (inSettings.mCLKOPin != ACAN2518FDSettings::SOF) {
      osc |= ((uint8_t) inSettings.mCLKOPin) << 5 ;
    }
    writeRegister8 (OSC_REGISTER, osc) ; // DS20005688B, page 16
  //--- Wait for PLL is ready (wait max 2 ms)
    if (pll != 0) {
      bool wait = true ;
      const uint32_t startTime = millis () ;
      while (wait) {
        wait = (readRegister8 (OSC_REGISTER + 1) & 0x1) == 0 ;  // DS20005688B, page 16
        if (wait && ((millis () - startTime) > 2)) {
          errorCode = kX10PLLNotReadyWithin1MS ;
          wait = false ;
        }
      }
    }
  }
//----------------------------------- Set full speed clock
// Upstream's rule, kept: 0.4 x SYSCLK. With a 40 MHz oscillator that is 16 MHz.
//
// ADDED IN THIS FORK: an explicit clamp to the MCP2518FD's absolute SPI maximum.
// It changes nothing at any oscillator this part supports (0.4 x 40 MHz is already
// under the ceiling); it is here so that the device limit is stated in the code
// rather than left as an emergent property of an arithmetic expression, and so a
// future change to the ratio cannot silently walk past it.
  {
    const uint32_t MCP2518FD_MAX_SPI_CLOCK = 20UL * 1000UL * 1000UL ; // DS20006027
    uint32_t spiClock = (inSettings.sysClock () * 2) / 5 ;
    if (spiClock > MCP2518FD_MAX_SPI_CLOCK) {
      spiClock = MCP2518FD_MAX_SPI_CLOCK ;
    }
    mSPISettings = SPISettings (spiClock, MSBFIRST, SPI_MODE0) ;
  }
//----------------------------------- Checking SPI connection is on (with a full speed clock)
//    We write and read back MCP2518FD RAM at address 0x400
  for (uint32_t i=1 ; (i != 0) && (errorCode == 0) ; i <<= 1) {
    writeRegister32 (0x400, i) ;
    const uint32_t readBackValue = readRegister32 (0x400) ;
    if (readBackValue != i) {
      errorCode = kReadBackErrorWithFullSpeedSPIClock ;
    }
  }
//----------------------------------- Install interrupt, configure external interrupt
  if (errorCode == 0) {
  //----------------------------------- Configure transmit and receive buffers
    mDriverTransmitBuffer.initWithSize (inSettings.mDriverTransmitFIFOSize) ;
    mDriverReceiveBuffer.initWithSize (inSettings.mDriverReceiveFIFOSize) ;
  //----------------------------------- Reset RAM
    for (uint16_t address = 0x400 ; address < 0xC00 ; address += 4) {
      writeRegister32 (address, 0) ;
    }
  //----------------------------------- Configure CLKO pin
    uint8_t data8 = 0x03 ; // Respect PM1-PM0 default values
    if (inSettings.mCLKOPin == ACAN2518FDSettings::SOF) {
      data8 |= 1 << 5 ; // SOF
    }
    if (inSettings.mTXCANIsOpenDrain) {
      data8 |= 1 << 4 ; // TXCANOD
    }
    if (inSettings.mINTIsOpenDrain) {
      data8 |= 1 << 6 ; // INTOD
    }
    writeRegister8 (IOCON_REGISTER_24_31, data8) ; // DS20005688B, page 24
  //----------------------------------- Configure ISO CRC Enable bit
    data8 = 1 << 6 ; // PXEDIS <-- 1
    if (inSettings.mISOCRCEnabled) {
      data8 |= 1 << 5 ; //  Enable ISO CRC in CAN FD Frames bit
    }
    writeRegister8 (CON_REGISTER, data8) ; // DS20005688B, page 24
  //----------------------------------- Configure DTC (DS20005688B, page 29)
    uint32_t data32 = 1UL << 25 ; // Enable Edge Filtering during Bus Integration state bit (added in 1.1.4)
    if (inSettings.mTDCO != 0) {
      data32 |= 1UL << 17 ; // Auto TDC
      const uint32_t TCDO = uint32_t (inSettings.mTDCO) & 0x7F ;
      data32 |= TCDO << 8 ;
    }
    writeRegister32 (TDC_REGISTER, data32) ;
  //----------------------------------- Configure TXQ
    data8 = inSettings.mControllerTXQBufferRetransmissionAttempts ;
    data8 <<= 5 ;
    data8 |= inSettings.mControllerTXQBufferPriority ;
    writeRegister8 (TXQCON_REGISTER + 2, data8) ; // DS20005688B, page 48
  // Bit 5-7: Payload Size bits
  // Bit 4-0: TXQ size
    mUsesTXQ = inSettings.mControllerTXQSize > 0 ;
    data8 = inSettings.mControllerTXQSize - 1 ;
    data8 |= inSettings.mControllerTXQBufferPayload << 5 ; // Payload
    writeRegister8 (TXQCON_REGISTER + 3, data8) ; // DS20005688B, page 48
    mTXQBufferPayload = ACAN2518FDSettings::objectSizeForPayload (inSettings.mControllerTXQBufferPayload) ;
  //----------------------------------- Configure TXQ and TEF
  // Bit 4: Enable Transmit Queue bit ---> 1: Enable TXQ and reserves space in RAM
  // Bit 3: Store in Transmit Event FIFO bit ---> 0: Don’t save transmitted messages in TEF
  // Bit 0: RTXAT ---> 1: Enable CiFIFOCONm.TXAT to control retransmission attempts
    data8 = 0x01 ; // Enable RTXAT to limit retransmissions (Flole)
    data8 |= mUsesTXQ ? (1 << 4) : 0x00 ; // Bug fix in 1.1.4 (thanks to danielhenz)
    writeRegister8 (CON_REGISTER + 2, data8) ; // DS20005688B, page 24
  //----------------------------------- Configure RX FIFO (FIFOCON, DS20005688B, page 52)
    data8 = inSettings.mControllerReceiveFIFOSize - 1 ; // Set receive FIFO size
    data8 |= inSettings.mControllerReceiveFIFOPayload << 5 ; // Payload
    writeRegister8 (FIFOCON_REGISTER (RECEIVE_FIFO_INDEX) + 3, data8) ;
    data8  = 1 << 0 ; // Interrupt Enabled for FIFO not Empty (TFNRFNIE)
    data8 |= 1 << 3 ; // Interrupt Enabled for FIFO Overflow (RXOVIE)
    writeRegister8 (FIFOCON_REGISTER (RECEIVE_FIFO_INDEX), data8) ;
    mReceiveFIFOPayload = ACAN2518FDSettings::objectSizeForPayload (inSettings.mControllerReceiveFIFOPayload) ;
  //----------------------------------- Configure TX FIFO (FIFOCON, DS20005688B, page 52)
    data8 = inSettings.mControllerTransmitFIFORetransmissionAttempts ;
    data8 <<= 5 ;
    data8 |= inSettings.mControllerTransmitFIFOPriority ;
    writeRegister8 (FIFOCON_REGISTER (TRANSMIT_FIFO_INDEX) + 2, data8) ;
    data8 = inSettings.mControllerTransmitFIFOSize - 1 ; // Set transmit FIFO size
    data8 |= inSettings.mControllerTransmitFIFOPayload << 5 ; // Payload
    writeRegister8 (FIFOCON_REGISTER (TRANSMIT_FIFO_INDEX) + 3, data8) ;
    data8 = 1 << 7 ; // FIFO is a Tx FIFO
    data8 |= 1 << 4 ; // TXATIE ---> 1: Enable Transmit Attempts Exhausted Interrupt
    writeRegister8 (FIFOCON_REGISTER (TRANSMIT_FIFO_INDEX), data8) ;
    mTransmitFIFOPayload = ACAN2518FDSettings::objectSizeForPayload (inSettings.mControllerTransmitFIFOPayload) ;
  //----------------------------------- Configure receive filters
    uint8_t filterIndex = 0 ;
    ACAN2518FDFilters::Filter * filter = inFilters.mFirstFilter ;
    mCallBackFunctionArray = new ACANFDCallBackRoutine [inFilters.filterCount ()] ;
    while (NULL != filter) {
      mCallBackFunctionArray [filterIndex] = filter->mCallBackRoutine ;
      writeRegister32 (MASK_REGISTER (filterIndex), filter->mFilterMask) ; // DS20005688B, page 61
      writeRegister32 (FLTOBJ_REGISTER (filterIndex), filter->mAcceptanceFilter) ; // DS20005688B, page 60
      data8 = 1 << 7 ; // Filter is enabled
      data8 |= 1 ; // Message matching filter is stored in FIFO1
      writeRegister8 (FLTCON_REGISTER (filterIndex), data8) ; // DS20005688B, page 58
      filter = filter->mNextFilter ;
      filterIndex += 1 ;
    }
  //----------------------------------- Activate interrupts (INT, DS20005688B page 34)
    data8  = (1 << 1) ; // Receive FIFO Interrupt Enable
    data8 |= (1 << 0) ; // Transmit FIFO Interrupt Enable
    writeRegister8 (INT_REGISTER + 2, data8) ;
    data8  = (1 << 2) ; // TXATIE ---> 1: Transmit Attempt Interrupt Enable bit
    writeRegister8 (INT_REGISTER + 3, data8) ;
  //----------------------------------- Program nominal bit rate (NBTCFG register)
  //  bits 31-24: BRP - 1
  //  bits 23-16: TSEG1 - 1
  //  bit 15: unused
  //  bits 14-8: TSEG2 - 1
  //  bit 7: unused
  //  bits 6-0: SJW - 1
    uint32_t data = inSettings.mBitRatePrescaler - 1 ;
    data <<= 8 ;
    data |= inSettings.mArbitrationPhaseSegment1 - 1 ;
    data <<= 8 ;
    data |= inSettings.mArbitrationPhaseSegment2 - 1 ;
    data <<= 8 ;
    data |= inSettings.mArbitrationSJW - 1 ;
    writeRegister32 (NBTCFG_REGISTER, data);
  //----------------------------------- Program data bit rate (DBTCFG register)
  //  bits 31-24: BRP - 1
  //  bits 23-21: unused
  //  bits 20-16: TSEG1 - 1
  //  bits 15-12: unused
  //  bits 11-8: TSEG2 - 1
  //  bits 7-4: unused
  //  bits 3-0: SJW - 1
    data = inSettings.mBitRatePrescaler - 1 ;
    data <<= 8 ;
    data |= inSettings.mDataPhaseSegment1 - 1 ;
    data <<= 8 ;
    data |= inSettings.mDataPhaseSegment2 - 1 ;
    data <<= 8 ;
    data |= inSettings.mDataSJW - 1 ;
    writeRegister32 (DBTCFG_REGISTER, data) ;
  //----------------------------------- Request mode (CON_REGISTER + 3, DS20005688B, page 24)
  //  bits 7-4: Transmit Bandwith Sharing Bits ---> 0
  //  bit 3: Abort All Pending Transmissions bit --> 0
    mTXBWS_RequestedMode = inSettings.mRequestedMode | (TXBWS << 4) ;
    writeRegister8 (CON_REGISTER + 3, mTXBWS_RequestedMode);
  //----------------------------------- Wait (10 ms max) until requested mode is reached
    bool wait = true ;
    const uint32_t startTime = millis () ;
    while (wait) {
     const uint8_t actualMode = (readRegister8 (CON_REGISTER + 2) >> 5) & 0x07 ;
      wait = actualMode != inSettings.mRequestedMode ;
      if (wait && ((millis () - startTime) > 10)) {
        errorCode |= kRequestedModeTimeOut ;
        wait = false ;
      }
    }
    #ifdef ARDUINO_ARCH_ESP32
      xTaskCreate (myESP32Task, "ACAN2518Handler", 1024, this, 16, &mESP32TaskHandle) ;
    #endif
    if (mINT != 255) { // 255 means interrupt is not used
      #ifdef ARDUINO_ARCH_ESP32
        attachInterrupt (itPin, inInterruptServiceRoutine, FALLING) ;
      #else
        mSPI.usingInterrupt (itPin) ; // usingInterrupt is not implemented in Arduino ESP32
        attachInterrupt (itPin, inInterruptServiceRoutine, LOW) ; // Thank to Flole998
      #endif
    }
  // If you begin() multiple times without constructor,
  // mHardwareTxFIFOFull = true will block the transmitter.
    mHardwareTxFIFOFull = false ;
    mHardwareReceiveBufferOverflowCount = 0 ;
  }
//---
  return errorCode ;
}

//------------------------------------------------------------------------------
//   end method (resets the MCP2518FD, deallocate buffers, and detach interrupt pin)
//------------------------------------------------------------------------------

bool ACAN2518FD::end (void) {
  mSPI.beginTransaction (mSPISettings) ;
    #ifdef ARDUINO_ARCH_ESP32
      taskDISABLE_INTERRUPTS () ;
    #else
      noInterrupts () ;
    #endif
  //--- Detach interrupt pin
    if (mINT != 255) { // 255 means interrupt is not used
      const int8_t itPin = digitalPinToInterrupt (mINT) ;
      detachInterrupt (itPin) ; // Available for ESP32 and Arduino
    }
  //--- Request configuration mode
    bool wait = true ;
    bool ok = false ;
    const uint32_t startTime = millis () ;
    while (wait) {
      writeRegister8Assume_SPI_transaction (CON_REGISTER + 3, 0x04 | (1 << 3)) ; // Request configuration mode, abort all transmissions
      const uint8_t actualMode = (readRegister8Assume_SPI_transaction (CON_REGISTER + 2) >> 5) & 0x07 ;
      ok = actualMode == 0x04 ;
      wait = !ok ;
      if (wait && ((millis () - startTime) > 2)) { // Wait (2 ms max) until the configuration mode is reached
        wait = false ;
      }
    }
  //--- Reset MCP2518FD
    assertCS () ;
      mSPI.transfer16 (0x00) ; // Reset instruction: 0x0000
    deassertCS () ;
  //--- ESP32: delete associated task
    #ifdef ARDUINO_ARCH_ESP32
      if (mESP32TaskHandle != nullptr) {
        vTaskDelete (mESP32TaskHandle) ;
        mESP32TaskHandle = nullptr ;
      }
    #endif
  //--- Deallocate buffers
    delete [] mCallBackFunctionArray ; mCallBackFunctionArray = nullptr ;
    mDriverReceiveBuffer.initWithSize (0) ;
    mDriverTransmitBuffer.initWithSize (0) ;
  //---
    #ifdef ARDUINO_ARCH_ESP32
      taskENABLE_INTERRUPTS () ;
    #else
      interrupts () ;
    #endif
  mSPI.endTransaction () ;
//---
  return ok ;
}

//------------------------------------------------------------------------------
//    SEND FRAME
//------------------------------------------------------------------------------

bool ACAN2518FD::tryToSend (const CANFDMessage & inMessage) {
  bool ok = inMessage.isValid () ;
  if (ok) {
    mSPI.beginTransaction (mSPISettings) ;
      #ifdef ARDUINO_ARCH_ESP32
        taskDISABLE_INTERRUPTS () ;
      #else
        noInterrupts () ;
      #endif
        if (inMessage.idx == 0) {
          ok = inMessage.len <= mTransmitFIFOPayload ;
          if (ok) {
            ok = enterInTransmitBuffer (inMessage) ;
          }
        }else if (inMessage.idx == 255) {
          ok = inMessage.len <= mTXQBufferPayload ;
          if (ok) {
            ok = sendViaTXQ (inMessage) ;
          }
        }
      #ifdef ARDUINO_ARCH_ESP32
        taskENABLE_INTERRUPTS () ;
      #else
        interrupts () ;
      #endif
    mSPI.endTransaction () ;
  }
  return ok ;
}

//------------------------------------------------------------------------------

bool ACAN2518FD::enterInTransmitBuffer (const CANFDMessage & inMessage) {
  bool result ;
  if (mHardwareTxFIFOFull) {
    result = mDriverTransmitBuffer.append (inMessage) ;
  }else{
    result = true ;
    appendInControllerTxFIFO (inMessage) ;
  //--- If controller FIFO is full, enable "FIFO not full" interrupt
    const uint8_t status = readRegister8Assume_SPI_transaction (FIFOSTA_REGISTER (TRANSMIT_FIFO_INDEX)) ;
    if ((status & 1) == 0) { // FIFO is full
      uint8_t data8 = 1 << 7 ;  // FIFO is a transmit FIFO
      data8 |= 1 ; // Enable "FIFO not full" interrupt
      data8 |= 1 << 4 ; // TXATIE ---> 1: Enable Transmit Attempts Exhausted Interrupt
      writeRegister8Assume_SPI_transaction (FIFOCON_REGISTER (TRANSMIT_FIFO_INDEX), data8) ;
      mHardwareTxFIFOFull = true ;
    }
  }
  return result ;
}

//------------------------------------------------------------------------------

static uint32_t lengthCodeForLength (const uint8_t inLength) {
  uint32_t result = inLength & 0x0F ;
  switch (inLength) {
    case 12 : result =  9 ; break ;
    case 16 : result = 10 ; break ;
    case 20 : result = 11 ; break ;
    case 24 : result = 12 ; break ;
    case 32 : result = 13 ; break ;
    case 48 : result = 14 ; break ;
    case 64 : result = 15 ; break ;
  }
  return result ;
}

//------------------------------------------------------------------------------

void ACAN2518FD::appendInControllerTxFIFO (const CANFDMessage & inMessage) {
  const uint16_t ramAddr = uint16_t (0x400 + readRegister32Assume_SPI_transaction (FIFOUA_REGISTER (TRANSMIT_FIFO_INDEX))) ;
//--- Write identifier: if an extended frame is sent, identifier bits sould be reordered (see DS20005678B, page 27)
  uint32_t idf = inMessage.id ;
  if (inMessage.ext) {
    idf = ((inMessage.id >> 18) & 0x7FF) | ((inMessage.id & 0x3FFFF) << 11) ;
  }
//--- Write DLC field, FDF, BRS, RTR, IDE bits
  uint32_t flags = lengthCodeForLength (inMessage.len) ;
  if (inMessage.ext) {
    flags |= 1 << 4 ; // Set EXT bit
  }
  switch (inMessage.type) {
  case CANFDMessage::CAN_REMOTE :
    flags |= 1 << 5 ; // Set RTR bit
    break ;
  case CANFDMessage::CAN_DATA :
   break ;
  case CANFDMessage::CANFD_NO_BIT_RATE_SWITCH :
    flags |= 1 << 7 ; // Set FDF bit
    break ;
  case CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH :
    flags |= 1 << 7 ; // Set FDF bit
    flags |= 1 << 6 ; // Set BRS bit
    break ;
  }
//--- Word count
  const uint32_t wordCount = (inMessage.len + 3) / 4 ;
//--- Write word register via 6-byte buffer (speed enhancement, thanks to thomasfla)
  uint8_t buffer [74] = {0} ;
//--- Enter command
  const uint16_t writeCommand = (ramAddr & 0x0FFF) | (0b0010 << 12) ;
  buffer [0] = writeCommand >> 8 ;
  buffer [1] = writeCommand & 0xFF ;
//--- Enter values
  enterU32InBufferAtIndex (idf, buffer, 2) ;
  enterU32InBufferAtIndex (flags, buffer, 6) ;
  for (uint32_t i=0 ; i < wordCount ; i++) {
    enterU32InBufferAtIndex (inMessage.data32 [i], buffer, 10 + 4 * i) ;
  }
//--- SPI transfer
  assertCS () ;
    mSPI.transfer (buffer, 10 + 4 * wordCount) ;
  deassertCS () ;
//--- Increment FIFO, send message (see DS20005688B, page 48)
  const uint8_t data8 = (1 << 0) | (1 << 1) ; // Set UINC bit, TXREQ bit
  writeRegister8Assume_SPI_transaction (FIFOCON_REGISTER (TRANSMIT_FIFO_INDEX) + 1, data8);
}

//------------------------------------------------------------------------------

bool ACAN2518FD::sendViaTXQ (const CANFDMessage & inMessage) {
  bool ok = mUsesTXQ ;
  if (ok) {
    uint8_t sta = readRegister8Assume_SPI_transaction (TXQSTA_REGISTER) ;
  //--- Check Transmit Attempts Exhausted Interrupt Pending bit
    ok = (sta & (1 << 4)) != 0 ;
    if (ok) {
      writeRegister8Assume_SPI_transaction (TXQSTA_REGISTER, ~ (1 << 4)) ;
    }else{
    //--- Enter message only if TXQ FIFO is not full (see DS20005688B, page 50)
      ok = (sta & 1) != 0 ;
    }
    if (ok) {
      const uint16_t ramAddress = (uint16_t) (0x400 + readRegister32Assume_SPI_transaction (TXQUA_REGISTER)) ;
    //--- Write identifier: if an extended frame is sent, identifier bits sould be reordered (see DS20005678B, page 27)
      uint32_t idf = inMessage.id ;
      if (inMessage.ext) {
        idf = ((inMessage.id >> 18) & 0x7FF) | ((inMessage.id & 0x3FFFF) << 11) ;
      }
    //--- Write DLC field, FDF, BRS, RTR, IDE bits
      uint32_t flags = lengthCodeForLength (inMessage.len) ;
      if (inMessage.ext) {
        flags |= 1 << 4 ; // Set EXT bit
      }
      switch (inMessage.type) {
      case CANFDMessage::CAN_REMOTE :
        flags |= 1 << 5 ; // Set RTR bit
        break ;
      case CANFDMessage::CAN_DATA :
       break ;
      case CANFDMessage::CANFD_NO_BIT_RATE_SWITCH :
        flags |= 1 << 7 ; // Set FDF bit
        break ;
      case CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH :
        flags |= 1 << 7 ; // Set FDF bit
        flags |= 1 << 6 ; // Set BRS bit
        break ;
      }
    //--- Word count
      const uint32_t wordCount = (inMessage.len + 3) / 4 ;
    //--- Transfer frame to the MCP2518FD
      uint8_t buffer [74] = {0} ;
    //--- Enter command
      const uint16_t writeCommand = (ramAddress & 0x0FFF) | (0b0010 << 12) ;
      buffer [0] = writeCommand >> 8 ;
      buffer [1] = writeCommand & 0xFF ;
    //--- Enter values
      enterU32InBufferAtIndex (idf, buffer, 2) ;
      enterU32InBufferAtIndex (flags, buffer, 6) ;
      for (uint32_t i=0 ; i < wordCount ; i++) {
        enterU32InBufferAtIndex (inMessage.data32 [i], buffer, 10 + 4 * i) ;
      }
    //--- SPI transfer
      assertCS () ;
        mSPI.transfer (buffer, 10 + 4 * wordCount) ;
      deassertCS () ;
    //--- Increment FIFO, send message (see DS20005688B, page 48)
      const uint8_t data8 = (1 << 0) | (1 << 1) ; // Set UINC bit, TXREQ bit
      writeRegister8Assume_SPI_transaction (TXQCON_REGISTER + 1, data8);
    }
  }
  return ok ;
}

//------------------------------------------------------------------------------
//    RECEIVE FRAME
//------------------------------------------------------------------------------

bool ACAN2518FD::available (void) {
  mSPI.beginTransaction (mSPISettings) ;
    #ifdef ARDUINO_ARCH_ESP32
      taskDISABLE_INTERRUPTS () ;
    #else
      noInterrupts () ;
    #endif
      const bool hasReceivedMessage = mDriverReceiveBuffer.count () > 0 ;
    #ifdef ARDUINO_ARCH_ESP32
      taskENABLE_INTERRUPTS () ;
    #else
      interrupts () ;
    #endif
  mSPI.endTransaction () ;
  return hasReceivedMessage ;
}

//------------------------------------------------------------------------------

bool ACAN2518FD::receive (CANFDMessage & outMessage) {
      const bool hasReceivedMessage = mDriverReceiveBuffer.remove (outMessage) ;
    //--- If receive interrupt is disabled, enable it (added in release 2.17)
      if (mINT == 255) { // No interrupt pin
        mRxInterruptEnabled = true ;
        isr_poll_core () ; // Perform polling
      }else if (!mRxInterruptEnabled) {

        mSPI.beginTransaction (mSPISettings) ;
          #ifdef ARDUINO_ARCH_ESP32
            taskDISABLE_INTERRUPTS () ;
          #else
            noInterrupts () ;
          #endif

        mRxInterruptEnabled = true ;
        uint8_t data8 = readRegister8Assume_SPI_transaction (INT_REGISTER + 2) ;
        data8 |= (1 << 1) ; // Receive FIFO Interrupt Enable
        writeRegister8Assume_SPI_transaction (INT_REGISTER + 2, data8) ;

        #ifdef ARDUINO_ARCH_ESP32
          taskENABLE_INTERRUPTS () ;
        #else
          interrupts () ;
        #endif
      mSPI.endTransaction () ;
      }
  return hasReceivedMessage ;
}

//------------------------------------------------------------------------------

bool ACAN2518FD::dispatchReceivedMessage (const tFilterMatchCallBack inFilterMatchCallBack) {
  CANFDMessage receivedMessage ;
  const bool hasReceived = receive (receivedMessage) ;
  if (hasReceived) {
    const uint32_t filterIndex = receivedMessage.idx ;
    if (NULL != inFilterMatchCallBack) {
      inFilterMatchCallBack (filterIndex) ;
    }
    ACANFDCallBackRoutine callBackFunction = (mCallBackFunctionArray == NULL) ? NULL : mCallBackFunctionArray [filterIndex] ;
    if (NULL != callBackFunction) {
      callBackFunction (receivedMessage) ;
    }
  }
  return hasReceived ;
}

//------------------------------------------------------------------------------
//    POLLING (ESP32)
//------------------------------------------------------------------------------

#ifdef ARDUINO_ARCH_ESP32
  void ACAN2518FD::poll (void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE ;
    xSemaphoreGiveFromISR (mISRSemaphore, &xHigherPriorityTaskWoken) ;
    portYIELD_FROM_ISR () ;
  }
#endif

//------------------------------------------------------------------------------
//    POLLING (other than ESP32)
//------------------------------------------------------------------------------

#ifndef ARDUINO_ARCH_ESP32
  void ACAN2518FD::poll (void) {
    noInterrupts () ;
      isr_poll_core () ;
    interrupts () ;
  }
#endif

//------------------------------------------------------------------------------
//   INTERRUPT SERVICE ROUTINE (ESP32)
// https://stackoverflow.com/questions/51750377/how-to-disable-interrupt-watchdog-in-esp32-or-increase-isr-time-limit
//------------------------------------------------------------------------------

#ifdef ARDUINO_ARCH_ESP32
  void ACAN2518FD::isr (void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE ;
    xSemaphoreGiveFromISR (mISRSemaphore, &xHigherPriorityTaskWoken) ;
    portYIELD_FROM_ISR () ;
  }
#endif

//------------------------------------------------------------------------------
//   INTERRUPT SERVICE ROUTINE (other than ESP32)
//------------------------------------------------------------------------------

#ifndef ARDUINO_ARCH_ESP32
  void ACAN2518FD::isr (void) {
    isr_poll_core () ;
  }
#endif

//------------------------------------------------------------------------------
//   INTERRUPT SERVICE ROUTINES (common)
//------------------------------------------------------------------------------

void ACAN2518FD::isr_poll_core (void) {
  mSPI.beginTransaction (mSPISettings) ;
    #ifdef ARDUINO_ARCH_ESP32
      taskDISABLE_INTERRUPTS () ;
    #endif
      bool handled = true ;
        while (handled) {
        handled = false ;
        const uint16_t it = readRegister16Assume_SPI_transaction (INT_REGISTER) ; // DS20005688B, page 34
        if (mRxInterruptEnabled && ((it & (1 << 1)) != 0)) { // Receive FIFO interrupt
          receiveInterrupt () ;
          handled = true ;
        }
        if ((it & (1 << 10)) != 0) { // Transmit Attempt interrupt
        //--- Clear Pending Transmit Attempt interrupt bit
          writeRegister8Assume_SPI_transaction (FIFOSTA_REGISTER (TRANSMIT_FIFO_INDEX), ~ (1 << 4)) ;
          transmitInterrupt () ;
          handled = true ;
        }else if ((it & (1 << 0)) != 0) { // Transmit FIFO interrupt
          transmitInterrupt () ;
          handled = true ;
        }
        if ((it & (1 << 2)) != 0) { // TBCIF interrupt
          writeRegister8Assume_SPI_transaction (INT_REGISTER, ~ (1 << 2)) ;
          handled = true ;
        }
        if ((it & (1 << 3)) != 0) { // MODIF interrupt
          writeRegister8Assume_SPI_transaction (INT_REGISTER, ~ (1 << 3)) ;
          handled = true ;
        }
        if ((it & (1 << 12)) != 0) { // SERRIF interrupt
          writeRegister8Assume_SPI_transaction (INT_REGISTER + 1, ~ (1 << 4)) ;
          handled = true ;
        }
        if ((it & (1 << 11)) != 0) { // RXOVIF interrupt
          handled = true ;
          if (mHardwareReceiveBufferOverflowCount < 255) {
            mHardwareReceiveBufferOverflowCount += 1 ;
          }
          writeRegister8Assume_SPI_transaction (FIFOSTA_REGISTER (RECEIVE_FIFO_INDEX), ~ (1 << 3)) ;
        }
      }
    #ifdef ARDUINO_ARCH_ESP32
      taskENABLE_INTERRUPTS () ;
    #endif
  mSPI.endTransaction () ;
}

//------------------------------------------------------------------------------

void ACAN2518FD::transmitInterrupt (void) { // Generated if hardware transmit FIFO is not full
  CANFDMessage message ;
  const bool hasMessage = mDriverTransmitBuffer.remove (message) ;
  if (hasMessage) {
    appendInControllerTxFIFO (message) ;
  }else{ // No message in transmit FIFO: disable "FIFO not full" interrupt
    uint8_t data8 = 1 << 7 ;  // FIFO is a transmit FIFO
    data8 |= 1 << 4 ; // TXATIE ---> 1: Enable Transmit Attempts Exhausted Interrupt
    writeRegister8Assume_SPI_transaction (FIFOCON_REGISTER (TRANSMIT_FIFO_INDEX), data8) ;
    mHardwareTxFIFOFull = false ;
  }
}

//------------------------------------------------------------------------------

void ACAN2518FD::receiveInterrupt (void) {
  const uint16_t ramAddress = uint16_t (0x400 + readRegister32Assume_SPI_transaction (FIFOUA_REGISTER (RECEIVE_FIFO_INDEX))) ;
  CANFDMessage message ;
//--- Read word register via 6-byte buffer (speed enhancement, thanks to thomasfla)
  uint8_t buffer [74] = {0} ;
//--- Enter command
  const uint16_t readCommand = (ramAddress & 0x0FFF) | (0b0011 << 12) ;
  buffer [0] = readCommand >> 8 ;
  buffer [1] = readCommand & 0xFF ;
//--- SPI transfer
  assertCS () ;
    mSPI.transfer (buffer, 74) ;
  //--- Read identifier (see DS20005678A, page 42)
    message.id = u32FromBufferAtIndex (buffer, 2) ;
  //--- Read DLC, RTR, IDE bits, and match filter index
    const uint32_t flags = u32FromBufferAtIndex (buffer, 6) ;
    static const uint8_t kLength [16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64} ;
    message.len = kLength [flags & 0x0F] ;
  //--- Write data (Swap data if processor is big endian)
    const uint32_t wordCount = (message.len + 3) / 4 ;
    for (uint32_t i=0 ; i < wordCount ; i++) {
      message.data32 [i] = u32FromBufferAtIndex (buffer, 10 + 4 * i) ;
    }
  deassertCS () ;
//--- Increment FIFO
  const uint8_t data8 = 1 << 0 ; // Set UINC bit (DS20005688B, page 52)
  writeRegister8Assume_SPI_transaction (FIFOCON_REGISTER (RECEIVE_FIFO_INDEX) + 1, data8) ;
  message.idx = uint8_t ((flags >> 11) & 0x1F) ;
//--- Message type (DS20005678B, page 42)
  if ((flags & (1 << 5)) != 0 ) { // RTR bit
    message.type = CANFDMessage::CAN_REMOTE ;
  }else if ((flags & (1 << 7)) == 0) { // FDF bit
    message.type = CANFDMessage::CAN_DATA ;
  }else if ((flags & (1 << 6)) == 0) { // BRS bit
    message.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH ;
  }else{
    message.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH ;
  }
//--- If an extended frame is received, identifier bits should be reordered (see DS20005678B, page 42)
  message.ext = (flags & (1 << 4)) != 0 ;
  if (message.ext) {
    const uint32_t tempID = message.id ;
    message.id = ((tempID >> 11) & 0x3FFFF) | ((tempID & 0x7FF) << 18) ;
  }
//--- Append message to driver receive FIFO
  mDriverReceiveBuffer.append (message) ;
//--- If mDriverReceiveBuffer is full, disable receive interrupt (added in release 2.17)
  if (mDriverReceiveBuffer.isFull ()) {
    mRxInterruptEnabled = false ;
    if (mINT != 255) {
      uint8_t data8 = readRegister8Assume_SPI_transaction (INT_REGISTER + 2) ;
      data8 &= ~ (1 << 1) ; // Receive FIFO Interrupt disable
      writeRegister8Assume_SPI_transaction (INT_REGISTER + 2, data8) ;
    }
  }
}

//------------------------------------------------------------------------------
//   MCP2518FD REGISTER ACCESS, SECOND LEVEL FUNCTIONS (HANDLE CS, ASSUME WITHIN SPI TRANSACTION)
//------------------------------------------------------------------------------

void ACAN2518FD::writeRegister32Assume_SPI_transaction (const uint16_t inRegisterAddress,
                                                        const uint32_t inValue) {
//--- Write word register via 6-byte buffer (speed enhancement, thanks to thomasfla)
  uint8_t buffer [6] = {0} ;
//--- Enter command
  const uint16_t writeCommand = (inRegisterAddress & 0x0FFF) | (0b0010 << 12) ;
  buffer [0] = writeCommand >> 8 ;
  buffer [1] = writeCommand & 0xFF ;
//--- Enter register value
  enterU32InBufferAtIndex (inValue, buffer, 2) ;
//--- SPI transfer
  assertCS () ;
    mSPI.transfer (buffer, 6) ;
  deassertCS () ;
}

//------------------------------------------------------------------------------

void ACAN2518FD::writeRegister8Assume_SPI_transaction (const uint16_t inRegisterAddress,
                                                       const uint8_t inValue) {
//--- Write byte register via 3-byte buffer (speed enhancement, thanks to thomasfla)
  uint8_t buffer [3] = {0} ;
  const uint16_t writeCommand = (inRegisterAddress & 0x0FFF) | (0b0010 << 12) ;
  buffer [0] = writeCommand >> 8 ;
  buffer [1] = writeCommand & 0xFF ;
  buffer [2] = inValue ;
  assertCS () ;
    mSPI.transfer (buffer, 3) ;
  deassertCS () ;
}

//------------------------------------------------------------------------------

uint32_t ACAN2518FD::readRegister32Assume_SPI_transaction (const uint16_t inRegisterAddress) {
//--- Read word register via 6-byte buffer (speed enhancement, thanks to thomasfla)
  uint8_t buffer [6] = {0} ;
//--- Enter command
  const uint16_t readCommand = (inRegisterAddress & 0x0FFF) | (0b0011 << 12) ;
  buffer [0] = readCommand >> 8 ;
  buffer [1] = readCommand & 0xFF ;
//--- SPI transfer
  assertCS () ;
    mSPI.transfer (buffer, 6) ;
  deassertCS () ;
//--- Get result
  const uint32_t result = u32FromBufferAtIndex (buffer, 2) ;
//---
  return result ;
}

//------------------------------------------------------------------------------

uint16_t ACAN2518FD::readRegister16Assume_SPI_transaction (const uint16_t inRegisterAddress) {
//--- Read half-word register via 4-byte buffer (speed enhancement, thanks to thomasfla)
  uint8_t buffer [4] = {0} ;
//--- Enter command
  const uint16_t readCommand = (inRegisterAddress & 0x0FFF) | (0b0011 << 12) ;
  buffer [0] = readCommand >> 8 ;
  buffer [1] = readCommand & 0xFF ;
//--- SPI transfer
  assertCS () ;
    mSPI.transfer (buffer, 4) ;
  deassertCS () ;
//--- Get result
  const uint16_t result = u16FromBufferAtIndex (buffer, 2) ;
//---
  return result ;
}

//------------------------------------------------------------------------------

uint8_t ACAN2518FD::readRegister8Assume_SPI_transaction (const uint16_t inRegisterAddress) {
//--- Read byte register via 3-byte buffer (speed enhancement, thanks to thomasfla)
  uint8_t buffer [3] = {0} ;
  const uint16_t readCommand = (inRegisterAddress & 0x0FFF) | (0b0011 << 12) ;
  buffer [0] = readCommand >> 8;
  buffer [1] = readCommand & 0xFF;
  assertCS () ;
    mSPI.transfer(buffer, 3) ;
  deassertCS () ;
  return buffer [2] ;
}

//------------------------------------------------------------------------------
//   MCP2518FD REGISTER ACCESS, THIRD LEVEL FUNCTIONS (HANDLE CS AND SPI TRANSACTION)
//------------------------------------------------------------------------------

void ACAN2518FD::writeRegister8 (const uint16_t inRegisterAddress, const uint8_t inValue) {
  mSPI.beginTransaction (mSPISettings) ;
    #ifdef ARDUINO_ARCH_ESP32
      taskDISABLE_INTERRUPTS () ;
    #else
      noInterrupts () ;
    #endif
      writeRegister8Assume_SPI_transaction (inRegisterAddress, inValue) ;
    #ifdef ARDUINO_ARCH_ESP32
      taskENABLE_INTERRUPTS () ;
    #else
      interrupts () ;
    #endif
  mSPI.endTransaction () ;
}

//------------------------------------------------------------------------------

uint8_t ACAN2518FD::readRegister8 (const uint16_t inRegisterAddress) {
  mSPI.beginTransaction (mSPISettings) ;
    #ifdef ARDUINO_ARCH_ESP32
      taskDISABLE_INTERRUPTS () ;
    #else
      noInterrupts () ;
    #endif
      const uint8_t result = readRegister8Assume_SPI_transaction (inRegisterAddress) ;
    #ifdef ARDUINO_ARCH_ESP32
      taskENABLE_INTERRUPTS () ;
    #else
      interrupts () ;
    #endif
  mSPI.endTransaction () ;
  return result ;
}

//------------------------------------------------------------------------------

uint16_t ACAN2518FD::readRegister16 (const uint16_t inRegisterAddress) {
  mSPI.beginTransaction (mSPISettings) ;
    #ifdef ARDUINO_ARCH_ESP32
      taskDISABLE_INTERRUPTS () ;
    #else
      noInterrupts () ;
    #endif
      const uint16_t result = readRegister16Assume_SPI_transaction (inRegisterAddress) ;
    #ifdef ARDUINO_ARCH_ESP32
      taskENABLE_INTERRUPTS () ;
    #else
      interrupts () ;
    #endif
  mSPI.endTransaction () ;
  return result ;
}

//------------------------------------------------------------------------------

void ACAN2518FD::writeRegister32 (const uint16_t inRegisterAddress, const uint32_t inValue) {
  mSPI.beginTransaction (mSPISettings) ;
    #ifdef ARDUINO_ARCH_ESP32
      taskDISABLE_INTERRUPTS () ;
    #else
      noInterrupts () ;
    #endif
      writeRegister32Assume_SPI_transaction (inRegisterAddress, inValue) ;
    #ifdef ARDUINO_ARCH_ESP32
      taskENABLE_INTERRUPTS () ;
    #else
      interrupts () ;
    #endif
  mSPI.endTransaction () ;
}

//------------------------------------------------------------------------------

uint32_t ACAN2518FD::readRegister32 (const uint16_t inRegisterAddress) {
  mSPI.beginTransaction (mSPISettings) ;
    #ifdef ARDUINO_ARCH_ESP32
      taskDISABLE_INTERRUPTS () ;
    #else
      noInterrupts () ;
    #endif
      const uint32_t result = readRegister32Assume_SPI_transaction (inRegisterAddress) ;
    #ifdef ARDUINO_ARCH_ESP32
      taskENABLE_INTERRUPTS () ;
    #else
      interrupts () ;
    #endif
  mSPI.endTransaction () ;
  return result ;
}

//------------------------------------------------------------------------------
//    Current MCP2518FD Operation Mode
//------------------------------------------------------------------------------

ACAN2518FDSettings::OperationMode ACAN2518FD::currentOperationMode (void) {
  const uint8_t mode = readRegister8 (CON_REGISTER + 2) >> 5 ;
  return ACAN2518FDSettings::OperationMode (mode) ;
}

//------------------------------------------------------------------------------

bool ACAN2518FD::recoverFromRestrictedOperationMode (void) {
   bool recoveryDone = false ;
   if (currentOperationMode () == ACAN2518FDSettings::RestrictedOperation) { // In Restricted Operation Mode
  //----------------------------------- Request mode (CON_REGISTER + 3)
  //  bits 7-4: Transmit Bandwith Sharing Bits ---> 0
  //  bit 3: Abort All Pending Transmission bits --> 0
    writeRegister8 (CON_REGISTER + 3, mTXBWS_RequestedMode);
  //----------------------------------- Wait (10 ms max) until requested mode is reached
    bool wait = true ;
    const uint32_t startTime = millis () ;
    while (wait) {
      const uint8_t actualMode = (readRegister8 (CON_REGISTER + 2) >> 5) & 0x07 ;
      wait = actualMode != (mTXBWS_RequestedMode & 0x07) ;
      recoveryDone = !wait ;
      if (wait && ((millis () - startTime) > 10)) {
        wait = false ;
      }
    }
  }
  return recoveryDone ;
}

//------------------------------------------------------------------------------
//    Set MCP2518FD Operation Mode
//------------------------------------------------------------------------------

void ACAN2518FD::setOperationMode (const ACAN2518FDSettings::OperationMode inOperationMode) {
//  bits 7-4: Transmit Bandwith Sharing Bits ---> 0
//  bit 3: Abort All Pending Transmission bits --> 0
  writeRegister8 (CON_REGISTER + 3, uint8_t (inOperationMode));
}

//------------------------------------------------------------------------------

void ACAN2518FD::reset2518FD (void) {
  mSPI.beginTransaction (mSPISettings) ; // Check RESET is performed with 800 kHz clock
    #ifdef ARDUINO_ARCH_ESP32
      taskDISABLE_INTERRUPTS () ;
    #else
      noInterrupts () ;
    #endif
      assertCS () ;
        mSPI.transfer16 (0x00) ; // Reset instruction: 0x0000
      deassertCS () ;
    #ifdef ARDUINO_ARCH_ESP32
      taskENABLE_INTERRUPTS () ;
    #else
      interrupts () ;
    #endif
  mSPI.endTransaction () ;
}

//------------------------------------------------------------------------------
//    Sleep Mode to Configuration Mode
// (returns true if MCP2518FD was in sleep mode)
//------------------------------------------------------------------------------
// The device exits Sleep mode due to a dominant edge on RXCAN or by enabling the oscillator (clearing OSC.OSCDIS).
// The module will transition automatically to Configuration mode.

bool ACAN2518FD::performSleepModeToConfigurationMode (void) {
  uint8_t value = readRegister8 (OSC_REGISTER) ;
  const bool inSleepMode = (value & (1 << 2)) != 0 ;
  if (inSleepMode) {
    value &= ~ (1 << 2) ; // Reset OSCDIS bit
    writeRegister8 (OSC_REGISTER, value) ;
  //--- Wait Clock is ready, ie OSC.OSCRDY is 1
    bool wait = true ;
    while (wait) {
      wait = (readRegister8 (OSC_REGISTER + 1) & (1 << 2)) == 0 ;
    }
  }
  return inSleepMode ;
}

//------------------------------------------------------------------------------
//    SLEEP / WAKE   (ADDED IN THIS FORK - see README.md)
//------------------------------------------------------------------------------
//  Everything below is new. It exists because an application that leaves these
//  controllers powered has to be able to park them and be woken by bus activity,
//  and the upstream driver has no API for either - not even a public register
//  accessor to build one on.
//------------------------------------------------------------------------------

bool ACAN2518FD::setOperationModeAndWait (const ACAN2518FDSettings::OperationMode inMode,
                                          const uint32_t inTimeoutMs) {
  setOperationMode (inMode) ;
  const uint32_t startTime = millis () ;
  for (;;) {
    if (uint8_t (currentOperationMode ()) == uint8_t (inMode)) {
      return true ;
    }
    if ((millis () - startTime) > inTimeoutMs) {
      return false ;
    }
  }
}

//------------------------------------------------------------------------------
//  THE ONE THAT MATTERS. begin() writes RXIE|TXIE to CiINT+2 and TXATIE to
//  CiINT+3, and leaves WAKIE (bit 30) clear. A controller slept as the library
//  leaves it DOES wake on a dominant edge at RXCAN - that transition is automatic
//  in silicon - but it asserts nothing on its INT pin, so a host watching that pin
//  sleeps straight through the event.
//
//  Clearing the other three enables is the mirror-image fix: while asleep the only
//  thing that should be able to pull INT low is a wake-up. An INT held low by an
//  ordinary RX or TX condition, at the moment the host arms a level-triggered
//  wake, is an instant wake - and then a sleep/wake loop that burns more current
//  than staying awake would have.
//------------------------------------------------------------------------------

void ACAN2518FD::maskInterruptsForSleep (void) {
  mSavedInterruptEnable2 = readRegister8 (INT_REGISTER + 2) ;
  mSavedInterruptEnable3 = readRegister8 (INT_REGISTER + 3) ;
  mInterruptsMaskedForSleep = true ;
  writeRegister8 (INT_REGISTER + 2, 0x00) ;        // no RXIE, no TXIE
  writeRegister8 (INT_REGISTER + 3, 1 << 6) ;      // WAKIE only (drops TXATIE)
  writeRegister8 (INT_REGISTER + 0, 0x00) ;        // clear pending flags...
  writeRegister8 (INT_REGISTER + 1, 0x00) ;        // ...WAKIF included
}

//------------------------------------------------------------------------------

void ACAN2518FD::restoreInterruptsAfterSleep (void) {
  if (!mInterruptsMaskedForSleep) {
    return ;
  }
  writeRegister8 (INT_REGISTER + 0, 0x00) ;
  writeRegister8 (INT_REGISTER + 1, 0x00) ;
  writeRegister8 (INT_REGISTER + 2, mSavedInterruptEnable2) ;
  writeRegister8 (INT_REGISTER + 3, mSavedInterruptEnable3) ;
  mInterruptsMaskedForSleep = false ;
}

//------------------------------------------------------------------------------
//  CiCON bits 8..10: WAKFIL (bit 8), WFT (bits 9-10). Writable in Configuration
//  mode; read back so the caller learns what the device actually has rather than
//  what it was asked for.
//------------------------------------------------------------------------------

bool ACAN2518FD::setWakeUpFilter (const bool inEnabled, const uint8_t inFilterTime) {
  uint8_t value = readRegister8 (CON_REGISTER + 1) ;
  value &= uint8_t (~0x07) ;                       // clear WAKFIL and WFT
  if (inEnabled) {
    value |= 0x01 ;                                // WAKFIL
    value |= uint8_t ((inFilterTime & 0x03) << 1) ; // WFT
  }
  writeRegister8 (CON_REGISTER + 1, value) ;
  const bool actuallyEnabled = (readRegister8 (CON_REGISTER + 1) & 0x01) != 0 ;
  return actuallyEnabled == inEnabled ;
}

//------------------------------------------------------------------------------

bool ACAN2518FD::isAsleep (void) {
  return (readRegister8 (OSC_REGISTER) & (1 << 2)) != 0 ; // OSCDIS
}

//------------------------------------------------------------------------------
//  Two witnesses, either one accepted: OPMOD reading Sleep, or OSCDIS set. How
//  OPMOD reads back through a stopped oscillator is not something worth staking a
//  power design on, and a false negative here means the system never sleeps at
//  all - a much worse outcome than a slightly loose check.
//------------------------------------------------------------------------------

bool ACAN2518FD::enterSleepMode (const uint32_t inTimeoutMs) {
  setOperationMode (ACAN2518FDSettings::Sleep) ;
  const uint32_t startTime = millis () ;
  for (;;) {
    if (uint8_t (currentOperationMode ()) == uint8_t (ACAN2518FDSettings::Sleep)) {
      return true ;
    }
    if (isAsleep ()) {
      return true ;
    }
    if ((millis () - startTime) > inTimeoutMs) {
      return false ;
    }
  }
}

//------------------------------------------------------------------------------
//  Safe to call BEFORE begin(): it brings CS up itself and drops to 800 kHz for
//  the transfers, the same conservative rate begin() uses around the reset, then
//  puts the previous SPI settings back. That is not a hypothetical case - a host
//  MCU reset does NOT reset this chip, so after a deep sleep every controller is
//  still asleep with its oscillator stopped, and begin() opens by requesting a
//  mode change and issuing a chip RESET against exactly that stopped oscillator.
//
//  Unlike performSleepModeToConfigurationMode(), the readiness wait is BOUNDED:
//  an absent or dead controller returns false instead of wedging the caller.
//------------------------------------------------------------------------------

bool ACAN2518FD::wakeUpFromSleepMode (const uint32_t inTimeoutMs) {
  initCS () ;
  deassertCS () ;
  const SPISettings savedSettings = mSPISettings ;
  mSPISettings = SPISettings (800UL * 1000, MSBFIRST, SPI_MODE0) ;

  bool oscillatorRunning = true ;
  uint8_t osc = readRegister8 (OSC_REGISTER) ;
  if ((osc & (1 << 2)) != 0) {                     // OSCDIS set: the part is asleep
    osc &= uint8_t (~(1 << 2)) ;
    writeRegister8 (OSC_REGISTER, osc) ;
    const uint32_t startTime = millis () ;
    while ((readRegister8 (OSC_REGISTER + 1) & (1 << 2)) == 0) {   // OSCRDY
      if ((millis () - startTime) > inTimeoutMs) {
        oscillatorRunning = false ;
        break ;
      }
    }
  }

  mSPISettings = savedSettings ;
  return oscillatorRunning ;
}

//------------------------------------------------------------------------------

uint32_t ACAN2518FD::errorCounters (void) {
  return readRegister32 (TREC_REGISTER) ;
}

//------------------------------------------------------------------------------

uint32_t ACAN2518FD::diagInfos (const int inIndex) { // thanks to Flole998 and turmary
  return readRegister32 (inIndex ? BDIAG1_REGISTER: BDIAG0_REGISTER) ;
}

//------------------------------------------------------------------------------
//    GPIO
//------------------------------------------------------------------------------

void ACAN2518FD::gpioSetMode (const uint8_t inPin, const uint8_t inMode) {
  if (inPin <= 1) {
    uint8_t value = readRegister8 (IOCON_REGISTER_00_07) ;
    if (inMode == INPUT) {
      value |=  (1 << inPin) ;
      if (inPin == 0) {
        value &= 3 ; // Clear XSBTYEN
      }
    }else if (inMode == OUTPUT) {
      value &= ~ (1 << inPin) ;
      if (inPin == 0) {
        value &= 3 ; // Clear XSBTYEN
      }
    }
    writeRegister8 (IOCON_REGISTER_00_07, value) ;
  }
}

//------------------------------------------------------------------------------

void ACAN2518FD::gpioWrite (const uint8_t inPin, const uint8_t inLevel) {
  if (inPin <= 1) {
    uint8_t value = readRegister8 (IOCON_REGISTER_08_15) ;
    if (inLevel == 0) { // LOW
      value &= ~ (1 << inPin) ;
    }else{
      value |=   (1 << inPin) ;
    }
    writeRegister8 (IOCON_REGISTER_08_15, value) ;
  }
}

//------------------------------------------------------------------------------

bool ACAN2518FD::gpioRead (const uint8_t inPin) {
  const uint8_t value = readRegister8 (IOCON_REGISTER_16_23) ;
  return (value >> inPin) & 1 ;
}

//------------------------------------------------------------------------------

void ACAN2518FD::configureGPIO0AsXSTBY (void) {
  uint8_t value = readRegister8 (IOCON_REGISTER_00_07) ;
  value |= (1 << 6) ; // Enable XSBTYEN
  writeRegister8 (IOCON_REGISTER_00_07, value) ;
}

//------------------------------------------------------------------------------
