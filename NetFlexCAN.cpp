// -------------------------------------------------------------
// a simple Arduino Teensy 3.1/3.2/3.5/3.6 CAN driver
// by teachop
// dual CAN support for MK66FX1M0 and updates for MK64FX512 by Pawelsky
// Interrupt driven Rx/Tx with buffers, object oriented callbacks by Collin Kidder
// RTR related code by H4nky84
// Statistics collection, timestamp and code clean-up my mdapoz
//
#include "NetFlexCAN.h"


#undef INCLUDE_FLEXCAN_DEBUG

#if defined(INCLUDE_FLEXCAN_DEBUG)
#define dbg_print(fmt, args...)     Serial.print (fmt , ## args)
#define dbg_println(fmt, args...)   Serial.println (fmt , ## args)
#else
#define dbg_print(fmt, args...)
#define dbg_println(fmt, args...)
#endif

// Supported FlexCAN interfaces

FlexCAN Can0(0);
#if defined(INCLUDE_FLEXCAN_CAN1)
FlexCAN Can1(1);
#if defined(INCLUDE_FLEXCAN_CAN2)
FlexCAN Can2(2);
#endif
#endif

#if F_CAN_CONSOLE
UHS_ByteBuffer blconsole_in;
blconsole_ Sercan;
void sercanEvent() {}
#endif

// default mask to apply to all mailboxes

CAN_filter_t FlexCAN::defaultMask;

// Some of these are complete guesses. Only really 8 and 16 have been validated.
// You have been warned. But, there aren't too many options for some of these

uint8_t bitTimingTable[21][3] = {
        // prop, seg1, seg2 (4 + prop + seg1 + seg2, seg2 must be at least 1)
        // No value can go over 7 here.
        {0, 0, 1}, //5
        {1, 0, 1}, //6
        {1, 1, 1}, //7
        {2, 1, 1}, //8
        {2, 2, 1}, //9
        {2, 3, 1}, //10
        {2, 3, 2}, //11
        {2, 4, 2}, //12
        {2, 5, 2}, //13
        {2, 5, 3}, //14
        {2, 6, 3}, //15
        {2, 7, 3}, //16
        {2, 7, 4}, //17
        {3, 7, 4}, //18
        {3, 7, 5}, //19
        {4, 7, 5}, //20
        {4, 7, 6}, //21
        {5, 7, 6}, //22
        {6, 7, 6}, //23
        {6, 7, 7}, //24
        {7, 7, 7}, //25
};

/**
 * FlexCAN class constructor
 *
 * @param id CAN bus interface selection
 */
FlexCAN::FlexCAN(uint8_t id) {
        if(id == 0) {
                flexcanBase = NFLEXCAN0_BASE;
        }
#if defined (INCLUDE_FLEXCAN_CAN1)
        else if(id == 1) {
                flexcanBase = NFLEXCAN1_BASE;
        }
#if defined (INCLUDE_FLEXCAN_CAN2)
        else if(id == 2) {
                flexcanBase = NFLEXCAN2_BASE;
        }
#endif
#endif
        uint32_t i;

        numTxMailboxes = 2;
#if defined(__MK20DX256__)
        IrqMessage = IRQ_CAN_MESSAGE;
        //IrqWarn = IRQ_CAN_WARN;
        //IrqError = IRQ_CAN_ERROR;
        //IrqWake = IRQ_CAN_WAKEUP;
#elif defined(__MK64FX512__)
        IrqMessage = IRQ_CAN0_MESSAGE;
        //IrqWarn = IRQ_CAN0_WARN;
        //IrqError = IRQ_CAN0_ERROR;
        //IrqWake = IRQ_CAN0_WAKEUP;
#elif defined(__MK66FX1M0__)
        if(flexcanBase == NFLEXCAN0_BASE) {
                IrqMessage = IRQ_CAN0_MESSAGE;
                //IrqWarn = IRQ_CAN0_WARN;
                //IrqError = IRQ_CAN0_ERROR;
                //IrqWake = IRQ_CAN0_WAKEUP;
        } else {
                IrqMessage = IRQ_CAN1_MESSAGE;
                //IrqWarn = IRQ_CAN1_WARN;
                //IrqError = IRQ_CAN1_ERROR;
                //IrqWake = IRQ_CAN1_WAKEUP;
        }
#elif defined(KINETISKE)
        //IrqWarn = IRQ_CAN0_WARN;
        //IrqError = IRQ_CAN0_ERROR;
        //IrqWake = IRQ_CAN0_WAKEUP;

        IrqMessage = IRQ_CAN0_MESSAGE;
#endif

        // Default mask is allow everything
        defaultMask.flags.remote = 0;
        defaultMask.flags.extended = 0;
        defaultMask.id = 0;
        sizeRxBuffer = SIZE_RX_BUFFER;
        sizeTxBuffer = SIZE_TX_BUFFER;
        tx_buffer = 0;
        rx_buffer = 0;
        // Initialize all message box specific ring buffers to 0.
        for(i = 0; i < getNumMailBoxes(); i++) {
                txRings[i] = 0;
        }

        // clear any listeners for received packets
        for(i = 0; i < SIZE_LISTENERS; i++) {
                listener[i] = NULL;
        }

        // clear statistics counts
        clearStats();
}

/**
 * Sets FlexCAN into freeze mode, disconnects the CAN bus
 */
void FlexCAN::end(void) {
        // enter freeze mode
        halt();
        NFLEXCANb_MCR(flexcanBase) |= (NFLEXCAN_MCR_HALT);
        while(!(NFLEXCANb_MCR(flexcanBase) & NFLEXCAN_MCR_FRZ_ACK));
}

/**
 * Initializes and starts FlexCAN
 *
 * @param baud 50000, 100000, 125000, 250000, 500000, 1000000
 * @param mask default mask to use for all mailbox masks. Optional.
 * @param txAlt 1 to enable alternate Tx pin (where available)
 * @param rxAlt 1 to enable alternate Rx pin (where available)
 */
void FlexCAN::begin(uint32_t baud, const CAN_filter_t &mask, uint8_t txAlt, uint8_t rxAlt) {
        // to-do: additional callback for extra settings
        initializeBuffers();

        // set up the pins
        setPins(txAlt, rxAlt);

#if defined(F_CAN)
        // Clocking should be already set up, already enabled elsewhere.
#else
        // select clock source 16MHz xtal
        OSC0_CR |= OSC_ERCLKEN;

        if(flexcanBase == NFLEXCAN0_BASE) {
                SIM_SCGC6 |= SIM_SCGC6_FLEXCAN0;
#if defined(INCLUDE_FLEXCAN_CAN1)
        } else if(flexcanBase == NFLEXCAN1_BASE) {
                SIM_SCGC3 |= SIM_SCGC3_FLEXCAN1;
#endif
        }
#endif

        FLEXCANb_CTRL1_SETUP(flexcanBase);

        // enable CAN
        NFLEXCANb_MCR(flexcanBase) |= NFLEXCAN_MCR_FRZ;
        NFLEXCANb_MCR(flexcanBase) &= ~NFLEXCAN_MCR_MDIS;

        while(NFLEXCANb_MCR(flexcanBase) & NFLEXCAN_MCR_LPM_ACK);

        // soft reset
        softReset();

        // wait for freeze ack
        waitFrozen();

        // disable self-reception
        NFLEXCANb_MCR(flexcanBase) |= NFLEXCAN_MCR_SRX_DIS;

        setBaudRate(baud);

        // enable per-mailbox filtering
        NFLEXCANb_MCR(flexcanBase) |= NFLEXCAN_MCR_IRMQ;

        // now have to set mask and filter for all the Rx mailboxes or they won't receive anything by default.
        for(uint8_t c = 0; c < getNumRxBoxes(); c++) {
                setMask(0, c);
                setFilter(mask, c);
        }
        NFLEXCANb_CTRL1(flexcanBase) |= NFLEXCAN_CTRL_LOM;
        // start the CAN
        exitHalt();
        waitReady();

        setNumTxBoxes(numTxMailboxes);

        NVIC_SET_PRIORITY(IrqMessage, IRQ_PRIORITY);
        NVIC_ENABLE_IRQ(IrqMessage);

        // enable interrupt masks for all 16 mailboxes
        NFLEXCANb_IMASK1(flexcanBase) = 0xFFFF;

        dbg_println("CAN initialized properly");
}

/**
 * Echo (self reception) enable/disable.
 *
 * @param mode
 */
void FlexCAN::setEcho(bool mode) {

}

/**
 * Set a mailbox buffer size
 * @param mbox Mail box
 * @param size in CAN_message_t
 */
void FlexCAN::setMailBoxTxBufferSize(uint8_t mbox, uint16_t size) {
        if(mbox >= getNumMailBoxes() || txRings[mbox] != 0) return;

        volatile CAN_message_t *buf = new CAN_message_t[size];
        txRings[mbox] = new CAN_frame_ringbuffer_t;
        initRingBuffer(*(txRings[mbox]), buf, size);
}

/**
 * Initialize dynamically sized buffers.
 */
void FlexCAN::initializeBuffers() {
        if(isInitialized()) return;

        // set up the transmit and receive ring buffers
        if(tx_buffer == 0) tx_buffer = new CAN_message_t[sizeTxBuffer];
        if(rx_buffer == 0) rx_buffer = new CAN_message_t[sizeRxBuffer];

        initRingBuffer(txRing, tx_buffer, sizeTxBuffer);
        initRingBuffer(rxRing, rx_buffer, sizeRxBuffer);
}

/**
 * Initialize FlexCAN pins.
 *
 * @param txAlt Alternate tx pin
 * @param rxAlt Alternate rx pin
 */
void FlexCAN::setPins(uint8_t txAlt, uint8_t rxAlt) {
        if(flexcanBase == NFLEXCAN0_BASE) {
                dbg_println("Begin setup of CAN0");
#if defined(KINETISKE)
                // No alt allowed. This is the programming interface default.
                CORE_PIN3_CONFIG = PORT_PCR_MUX(5);
                CORE_PIN4_CONFIG = PORT_PCR_MUX(5);
#elif defined(__MK66FX1M0__) || defined(__MK64FX512__)
                //  3=PTA12=CAN0_TX,  4=PTA13=CAN0_RX (default)
                // 29=PTB18=CAN0_TX, 30=PTB19=CAN0_RX (alternative)

                if(txAlt == 1)
                        CORE_PIN29_CONFIG = PORT_PCR_MUX(2);
                else
                        CORE_PIN3_CONFIG = PORT_PCR_MUX(2);

                // | PORT_PCR_PE | PORT_PCR_PS;

                if(rxAlt == 1)
                        CORE_PIN30_CONFIG = PORT_PCR_MUX(2);
                else
                        CORE_PIN4_CONFIG = PORT_PCR_MUX(2);
#else
                //  3=PTA12=CAN0_TX,  4=PTA13=CAN0_RX (default)
                // 32=PTB18=CAN0_TX, 25=PTB19=CAN0_RX (alternative)

                if(txAlt == 1)
                        CORE_PIN32_CONFIG = PORT_PCR_MUX(2);
                else
                        CORE_PIN3_CONFIG = PORT_PCR_MUX(2);

                // | PORT_PCR_PE | PORT_PCR_PS;

                if(rxAlt == 1)
                        CORE_PIN25_CONFIG = PORT_PCR_MUX(2);
                else
                        CORE_PIN4_CONFIG = PORT_PCR_MUX(2);
#endif
        }
#if defined(INCLUDE_FLEXCAN_CAN1)
        else if(flexcanBase == NFLEXCAN1_BASE) {
                dbg_println("Begin setup of CAN1");

                // 33=PTE24=CAN1_TX, 34=PTE25=CAN1_RX (default)
                // NOTE: Alternative CAN1 pins are not broken out on Teensy 3.6
                CORE_PIN33_CONFIG = PORT_PCR_MUX(2);
                CORE_PIN34_CONFIG = PORT_PCR_MUX(2); // | PORT_PCR_PE | PORT_PCR_PS;
        }
#endif
}

/**
 * Calculate and set baud rate as close as possible to what has been requested.
 *
 * @param baud desired baud rate
 */
void FlexCAN::setBaudRate(uint32_t baud) {
        // have to find a divisor that ends up as close to the target baud as possible while keeping the end result between 5 and 25

        dbg_println("Set baud rate");

        /*
         * Strategy that uses a system which attempts to generate a valid baud setting.
         * - You can freely divide the clock by anything from 1 to 256
         * - There is always a start bit (+1)
         * - The rest (prop, seg1, seg2) are specified 1 less than their actual value (aka +1)
         * - This gives the low end bit timing as 5 (1 + 1 + 2 + 1) and the high end 25 (1 + 8 + 8 + 8)
         * Example:
         *   16Mhz clock, divisor = 19+1, bit values add up to 16 = 16Mhz / 20 / 16 = 50k baud
         */
        uint32_t divisor = 0;
        uint32_t bestDivisor = 0;
        uint32_t result = FLEXCAN_BASE_FREQ / baud / (divisor + 1);
        int error = baud - (FLEXCAN_BASE_FREQ / (result * (divisor + 1)));
        int bestError = error;

        while(result > 5) {
                divisor++;
                result = FLEXCAN_BASE_FREQ / baud / (divisor + 1);

                if(result <= 25) {
                        error = baud - (FLEXCAN_BASE_FREQ / (result * (divisor + 1)));

                        if(error < 0)
                                error *= -1;

                        // if this error is better than we've ever seen then use it - it's the best option

                        if(error < bestError) {
                                bestError = error;
                                bestDivisor = divisor;
                        }

                        // If this is equal to a previously good option then
                        // switch to it but only if the bit time result was in the middle of the range
                        // this biases the output to use the middle of the range all things being equal
                        // Otherwise it might try to use a higher divisor and smaller values for prop, seg1, seg2
                        // and that's not necessarily the best idea.

                        if((error == bestError) && (result > 11) && (result < 19)) {
                                bestError = error;
                                bestDivisor = divisor;
                        }
                }
        }

        divisor = bestDivisor;
        result = FLEXCAN_BASE_FREQ / baud / (divisor + 1);

        if((result < 5) || (result > 25) || (bestError > 300)) {
                Serial.println("Abort in CAN begin. Couldn't find a suitable baud config!");
                return;
        }

        result -= 5; // the bitTimingTable is offset by 5 since there was no reason to store bit timings for invalid numbers
        uint8_t propSeg = bitTimingTable[result][0];
        uint8_t pSeg1 = bitTimingTable[result][1];
        uint8_t pSeg2 = bitTimingTable[result][2];

        // baud rate debug information
        dbg_println(" Bit time values:");
        dbg_print("  Prop = ");
        dbg_println(propSeg + 1);
        dbg_print("  Seg1 = ");
        dbg_println(pSeg1 + 1);
        dbg_print("  Seg2 = ");
        dbg_println(pSeg2 + 1);
        dbg_print("  Divisor = ");
        dbg_println(divisor + 1);

        NFLEXCANb_CTRL1(flexcanBase) = (NFLEXCAN_CTRL_PROPSEG(propSeg) | NFLEXCAN_CTRL_RJW(1) | NFLEXCAN_CTRL_ERR_MSK |
                NFLEXCAN_CTRL_PSEG1(pSeg1) | NFLEXCAN_CTRL_PSEG2(pSeg2) | NFLEXCAN_CTRL_PRESDIV(divisor));
        baud_rate = baud;
}

/*
 * \brief Halts CAN bus.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void FlexCAN::halt() {
        NFLEXCANb_MCR(flexcanBase) |= (NFLEXCAN_MCR_HALT);
        waitFrozen();
}

/*
 * \brief Exits from hat state.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void FlexCAN::exitHalt() {
        // exit freeze mode and wait until it is unfrozen.
        dbg_println("Exit halt");
        NFLEXCANb_MCR(flexcanBase) &= ~(NFLEXCAN_MCR_HALT);
        waitNotFrozen();
}

/*
 * \brief Makes CAN bus soft reset.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void FlexCAN::softReset() {
        dbg_println("Soft reset");
        NFLEXCANb_MCR(flexcanBase) ^= NFLEXCAN_MCR_SOFT_RST;

        while(NFLEXCANb_MCR(flexcanBase) & NFLEXCAN_MCR_SOFT_RST);
}

/*
 * \brief Freezes CAN bus.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void FlexCAN::freeze() {
        NFLEXCANb_MCR(flexcanBase) |= NFLEXCAN_MCR_FRZ;
}

/*
 * \brief Waits until CAN bus is frozen
 *
 * \param None.
 *
 * \retval None.
 *
 */

void FlexCAN::waitFrozen() {
        // wait for freeze ack
        dbg_println("Wait frozen");
        while(!isFrozen());
}

/*
 * \brief Waits until CAN bus is not frozen.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void FlexCAN::waitNotFrozen() {
        // wait for freeze ack
        dbg_println("Wait not frozen");
        while(isFrozen());
}

/*
 * \brief Waits until CAN bus is ready
 *
 * \param None.
 *
 * \retval None.
 *
 */

void FlexCAN::waitReady() {
        while(NFLEXCANb_MCR(flexcanBase) & NFLEXCAN_MCR_NOT_RDY);
}

/*
 * \brief Tests is CAN bus frozen.
 *
 * \param None.
 *
 * \retval true, if CAN bus is frozen.
 *
 */

bool FlexCAN::isFrozen() {
        return (NFLEXCANb_MCR(flexcanBase) & NFLEXCAN_MCR_FRZ_ACK);
}

/*
 * \brief Set listen only mode on or off.
 *
 * \param mode - set listen only mode?
 *
 * \retval None.
 *
 */

void FlexCAN::setListenOnly(bool mode) {
        // enter freeze mode if not already there

        if(!(NFLEXCANb_MCR(flexcanBase) & NFLEXCAN_MCR_FRZ_ACK)) {
                NFLEXCANb_MCR(flexcanBase) |= NFLEXCAN_MCR_FRZ;
                NFLEXCANb_MCR(flexcanBase) |= NFLEXCAN_MCR_HALT;
                while(!(NFLEXCANb_MCR(flexcanBase) & NFLEXCAN_MCR_FRZ_ACK));
        }

        if(mode)
                NFLEXCANb_CTRL1(flexcanBase) |= NFLEXCAN_CTRL_LOM;
        else
                NFLEXCANb_CTRL1(flexcanBase) &= ~NFLEXCAN_CTRL_LOM;

        // exit freeze mode and wait until it is unfrozen.

        NFLEXCANb_MCR(flexcanBase) &= ~NFLEXCAN_MCR_HALT;

        while(NFLEXCANb_MCR(flexcanBase) & NFLEXCAN_MCR_FRZ_ACK);
}

void FlexCAN::ChangeBaudRate(uint32_t baud) {
        // enter freeze mode if not already there

        if(!(NFLEXCANb_MCR(flexcanBase) & NFLEXCAN_MCR_FRZ_ACK)) {
                NFLEXCANb_MCR(flexcanBase) |= NFLEXCAN_MCR_FRZ;
                NFLEXCANb_MCR(flexcanBase) |= NFLEXCAN_MCR_HALT;
                while(!(NFLEXCANb_MCR(flexcanBase) & NFLEXCAN_MCR_FRZ_ACK));
        }

        setBaudRate(baud);
        // exit freeze mode and wait until it is unfrozen.

        NFLEXCANb_MCR(flexcanBase) &= ~NFLEXCAN_MCR_HALT;

        while(NFLEXCANb_MCR(flexcanBase) & NFLEXCAN_MCR_FRZ_ACK);

}

/*
 * \brief Initializes mailboxes to the requested mix of Rx and Tx boxes
 *
 * \param txboxes - How many of the 8 boxes should be used for Tx
 *
 * \retval number of tx boxes set.
 *
 */

uint8_t FlexCAN::setNumTxBoxes(uint8_t txboxes) {
        uint8_t c;
        uint32_t oldIde;

        if(txboxes > getNumMailBoxes() - 1) txboxes = getNumMailBoxes() - 1;
        if(txboxes < 1) txboxes = 1;

        numTxMailboxes = txboxes;

        if(!isInitialized()) return numTxMailboxes; // Just set the numTxMailboxes. Begin() will do final initialization.

        // Inialize Rx boxen
        for(c = 0; c < getNumRxBoxes(); c++) {
                // preserve the existing filter ide setting
                oldIde = NFLEXCANb_MBn_CS(flexcanBase, c) & NFLEXCAN_MB_CS_IDE;
                NFLEXCANb_MBn_CS(flexcanBase, c) = NFLEXCAN_MB_CS_CODE(NFLEXCAN_MB_CODE_RX_EMPTY) | oldIde;
        }

        // Initialize Tx boxen
        for(c = getFirstTxBox(); c < getNumMailBoxes(); c++) {
                NFLEXCANb_MBn_CS(flexcanBase, c) = NFLEXCAN_MB_CS_CODE(NFLEXCAN_MB_CODE_TX_INACTIVE);
        }

        return (numTxMailboxes);
}

/*
 * \brief Sets a per-mailbox filter. Sets both the storage and the actual mailbox.
 *
 * \param filter - a filled out filter structure
 * \param mbox - the mailbox to update
 *
 * \retval Nothing
 *
 */

void FlexCAN::setFilter(const CAN_filter_t &filter, uint8_t mbox) {
        if(mbox < getNumRxBoxes()) {
                MBFilters[mbox] = filter;

                if(filter.flags.extended) {
                        NFLEXCANb_MBn_ID(flexcanBase, mbox) = (filter.id & NFLEXCAN_MB_ID_EXT_MASK);
                        NFLEXCANb_MBn_CS(flexcanBase, mbox) |= NFLEXCAN_MB_CS_IDE;
                } else {
                        NFLEXCANb_MBn_ID(flexcanBase, mbox) = NFLEXCAN_MB_ID_IDSTD(filter.id);
                        NFLEXCANb_MBn_CS(flexcanBase, mbox) &= ~NFLEXCAN_MB_CS_IDE;
                }
        }
}

/*
 * \brief Gets a per-mailbox filter.
 *
 * \param filter - returned filter structure
 * \param mbox - mailbox selected
 *
 * \retval true if mailbox s valid, false otherwise
 *
 */

bool FlexCAN::getFilter(CAN_filter_t &filter, uint8_t mbox) {
        if(mbox < getNumRxBoxes()) {
                filter.id = MBFilters[mbox].id;
                filter.flags.extended = MBFilters[mbox].flags.extended;
                filter.flags.remote = MBFilters[mbox].flags.remote;
                filter.flags.reserved = MBFilters[mbox].flags.reserved;
                return (true);
        }

        return (false);
}

/*
 * \brief Set the mailbox mask for filtering packets
 *
 * \param mask - mask to apply.
 * \param mbox - mailbox number
 *
 * \retval None.
 */

void FlexCAN::setMask(uint32_t mask, uint8_t mbox) {
        if(mbox < getNumRxBoxes()) {

                /* Per mailbox masks can only be set in freeze mode so have to enter that mode if not already there. */
                bool wasFrozen = isFrozen();

                if(!wasFrozen) {
                        freeze();
                        halt();
                }

                NFLEXCANb_MB_MASK(flexcanBase, mbox) = mask;
                if(!wasFrozen) exitHalt();
        }
}

/*
 * \brief How many messages are available to read.
 *
 * \param None
 *
 * \retval A count of the number of messages available.
 */

uint32_t FlexCAN::available(void) {
        irqLock();
        uint32_t result = (ringBufferCount(rxRing));
        irqRelease();
        return result;
}

int FlexCAN::availableForWrite(void) {
        irqLock();
        uint32_t result = sizeTxBuffer - (ringBufferCount(txRing));
        irqRelease();
        return result;
}

int FlexCAN::availableForWrite(uint8_t mbox) {
        int result = 0;
        if(!isTxBox(mbox)) return result;

        irqLock();
        if(txRings[mbox] != 0) {
                result = ringBufferFree(*txRings[mbox]);
        }
        irqRelease();
        return result;
}

/*
 * \brief Clear the collected statistics
 *
 * \param None
 *
 * \retval None
 */

#if defined(COLLECT_CAN_STATS)

void FlexCAN::clearStats(void) {
        // initialize the statistics structure

        memset(&stats, 0, sizeof (stats));
        stats.enabled = false;
        stats.ringRxMax = SIZE_RX_BUFFER - 1;
        stats.ringTxMax = SIZE_TX_BUFFER - 1;
        stats.ringRxFramesLost = 0;
}
#endif

/*
 * \brief Retrieve a frame from the RX buffer
 *
 * \param msg - buffer reference to the frame structure to fill out
 *
 * \retval 0 no frames waiting to be received, 1 if a frame was returned
 */

int FlexCAN::read(CAN_message_t &msg) {
        /* pull the next available message from the ring */

        int result = 0;

        irqLock();
        if(removeFromRingBuffer(rxRing, msg) == true) {
                result = 1;
        }
        irqRelease();

        return result;
}

void FlexCAN::flush(uint8_t n) {
        bool flushed;
        if(!isTxBox(n)) return;
        if(txRings[n] != 0) {
                while(!isRingBufferEmpty(*txRings[n])) yield();

        }
        while(!isRingBufferEmpty(txRing)) yield();
        do {
                flushed = true;
                if(NFLEXCAN_get_code(NFLEXCANb_MBn_CS(flexcanBase, n)) != NFLEXCAN_MB_CODE_TX_INACTIVE) {
                        flushed = false;
                        break;
                }
        } while(!flushed);
}

void FlexCAN::flush(void) {
        uint32_t index = getNumMailBoxes();
        bool flushed;
        while(!isRingBufferEmpty(txRing)) yield();
        do {
                flushed = true;
                for(index = getFirstTxBox(); index < getNumMailBoxes(); index++) {
                        if(usesGlobalTxRing(index) && NFLEXCAN_get_code(NFLEXCANb_MBn_CS(flexcanBase, index)) != NFLEXCAN_MB_CODE_TX_INACTIVE) {
                                flushed = false;
                                break;
                        }
                }
        } while(!flushed);
}

/*
 * \brief Send a frame out of this canbus port
 *
 * \param msg - the filled out frame structure to use for sending
 *
 * \note Will do one of two things - 1. Send the given frame out of the first available mailbox
 * or 2. queue the frame for sending later via interrupt. Automatically turns on TX interrupt
 * if necessary.
 * Messages may be transmitted out of order, if more than one transmit mailbox is enabled.
 * The message queue ignores the message priority.
 *
 * Returns whether sending/queueing succeeded. Will not smash the queue if it gets full.
 */

int FlexCAN::write(const CAN_message_t &msg) {
        uint32_t index = getNumMailBoxes();
        int result = 0;

        irqLock();
        if(isRingBufferEmpty(txRing)) { // If there is nothing buffered, find free mailbox

                for(index = getFirstTxBox(); index < getNumMailBoxes(); index++) {
                        if(usesGlobalTxRing(index) && NFLEXCAN_get_code(NFLEXCANb_MBn_CS(flexcanBase, index)) == NFLEXCAN_MB_CODE_TX_INACTIVE) {
                                break; // found one
                        }
                }
        }
        if(index < getNumMailBoxes()) {
                dbg_println("Writing a frame directly.");

                writeTxRegisters(msg, index);
                result = 1;
        } else {
                // no mailboxes available. Try to buffer it

                if(addToRingBuffer(txRing, msg) == true) {
                        result = 1;
                }
                // else could not send the frame!
        }
        irqRelease();

        return result;
}

/*
 * \brief Send a frame out of this canbus port, using a specific mailbox. The TX queue is not used.
 *
 * \param msg - the filled out frame structure to use for sending
 * \param mbox - mailbox selected
 *
 * \note If the mailbox is available, the message is placed in the mailbox. The CAN controller
 * selects the next message to send from all filled transmit mailboxes, based on the priority.
 * This method allows callers to not use the transmit queue and prioritize messages by using
 * different mailboxes for different priority levels.
 * Using the same mailbox for a group of messages enforces the transmit order for this group.
 *
 * Returns whether the message was placed in the mailbox for sending.
 */

int FlexCAN::write(const CAN_message_t &msg, uint8_t mbox) {
        int result = 0;

        if(!isTxBox(mbox)) return result;

        irqLock();
        if(txRings[mbox] == 0 || isRingBufferEmpty(*txRings[mbox])) {
                if(NFLEXCAN_get_code(NFLEXCANb_MBn_CS(flexcanBase, mbox)) == NFLEXCAN_MB_CODE_TX_INACTIVE) {
                        writeTxRegisters(msg, mbox);
                        result = 1;
                }
        }
        if(result == 0 && txRings[mbox] != 0) {
                result = (addToRingBuffer(*txRings[mbox], msg) == true);
        }
        irqRelease();

        return result;
}


/*
 * \brief Send a frame out of this canbus port, using a specific mailbox. The TX queue is not used.
 * If a buffer exists, and has any content, it is coalesced to the last waiting packet if the same ID.
 * The idea is to take short packets used as streams, combine them and reduce small packet amounts.
 * Having a buffer, while not enforced is recommended, else smaller packets happen defeating the idea.
 * If the buffer is initially empty, a short packet can occur, and is normal.
 *
 * \param msg - the filled out frame structure to use for sending
 * \param mbox - mailbox selected
 *
 * \note If the mailbox is available, the message is placed in the mailbox. The CAN controller
 * selects the next message to send from all filled transmit mailboxes, based on the priority.
 * This method allows callers to not use the transmit queue and prioritize messages by using
 * different mailboxes for different priority levels.
 * Using the same mailbox for a group of messages enforces the transmit order for this group.
 *
 * Returns whether the message was placed in the mailbox for sending.
 */

int FlexCAN::Cwrite(const CAN_message_t &msg, uint8_t mbox) {
        int result = 0;

        if(!isTxBox(mbox)) return result;

        irqLock();
        if(txRings[mbox] == 0 || isRingBufferEmpty(*txRings[mbox])) {
                if(NFLEXCAN_get_code(NFLEXCANb_MBn_CS(flexcanBase, mbox)) == NFLEXCAN_MB_CODE_TX_INACTIVE) {
                        writeTxRegisters(msg, mbox);
                        result = 1;
                }
        }
        if(result == 0 && txRings[mbox] != 0) {
                // colesce if possible.
                result = (coalesceToRingBuffer(*txRings[mbox], msg) == true);
        }
        irqRelease();

        return result;
}

/*
 * \brief Write CAN message to the FlexCAN hardware registers.
 *
 * \param msg    - message structure to send.
 * \param buffer - mailbox number to write to.
 *
 * \retval None.
 *
 */

void FlexCAN::writeTxRegisters(const CAN_message_t &msg, uint8_t buffer) {
        // transmit the frame
        //    Commented below by TTL. That caused lock time to time to FLEXCAN_MB_CODE_TX_ONCE state.
        //    FLEXCANb_MBn_CS(flexcanBase, buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
        if(msg.flags.extended) {
                NFLEXCANb_MBn_ID(flexcanBase, buffer) = (msg.id & NFLEXCAN_MB_ID_EXT_MASK);
        } else {
                NFLEXCANb_MBn_ID(flexcanBase, buffer) = NFLEXCAN_MB_ID_IDSTD(msg.id);
        }

        NFLEXCANb_MBn_WORD0(flexcanBase, buffer) = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
        NFLEXCANb_MBn_WORD1(flexcanBase, buffer) = (msg.buf[4] << 24) | (msg.buf[5] << 16) | (msg.buf[6] << 8) | msg.buf[7];

        if(msg.flags.extended) {
                if(msg.flags.remote) {
                        NFLEXCANb_MBn_CS(flexcanBase, buffer) = NFLEXCAN_MB_CS_CODE(NFLEXCAN_MB_CODE_TX_ONCE) |
                                NFLEXCAN_MB_CS_LENGTH(msg.len) | NFLEXCAN_MB_CS_SRR |
                                NFLEXCAN_MB_CS_IDE | NFLEXCAN_MB_CS_RTR;
                } else {
                        NFLEXCANb_MBn_CS(flexcanBase, buffer) = NFLEXCAN_MB_CS_CODE(NFLEXCAN_MB_CODE_TX_ONCE) |
                                NFLEXCAN_MB_CS_LENGTH(msg.len) | NFLEXCAN_MB_CS_SRR |
                                NFLEXCAN_MB_CS_IDE;
                }
        } else {
                if(msg.flags.remote) {
                        NFLEXCANb_MBn_CS(flexcanBase, buffer) = NFLEXCAN_MB_CS_CODE(NFLEXCAN_MB_CODE_TX_ONCE) |
                                NFLEXCAN_MB_CS_LENGTH(msg.len) | NFLEXCAN_MB_CS_RTR;
                } else {
                        NFLEXCANb_MBn_CS(flexcanBase, buffer) = NFLEXCAN_MB_CS_CODE(NFLEXCAN_MB_CODE_TX_ONCE) |
                                NFLEXCAN_MB_CS_LENGTH(msg.len);
                }
        }
}

/*
 * \brief Read CAN message from the FlexCAN hardware registers.
 *
 * \param msg    - message structure to fill.
 * \param buffer - mailbox number to read from.
 *
 * \retval None.
 *
 */

void FlexCAN::readRxRegisters(CAN_message_t& msg, uint8_t buffer) {
        uint32_t mb_CS = NFLEXCANb_MBn_CS(flexcanBase, buffer);

        // get identifier and dlc
        msg.len = NFLEXCAN_get_length(mb_CS);
        msg.flags.extended = (mb_CS & NFLEXCAN_MB_CS_IDE) ? 1 : 0;
        msg.flags.remote = (mb_CS & NFLEXCAN_MB_CS_RTR) ? 1 : 0;
        msg.timestamp = NFLEXCAN_get_timestamp(mb_CS);
        msg.flags.overrun = 0;
        msg.flags.reserved = 0;
        msg.id = (NFLEXCANb_MBn_ID(flexcanBase, buffer) & NFLEXCAN_MB_ID_EXT_MASK);

        if(!msg.flags.extended) {
                msg.id >>= NFLEXCAN_MB_ID_STD_BIT_NO;
        }

        // check for mailbox buffer overruns
        if(NFLEXCAN_get_code(mb_CS) == NFLEXCAN_MB_CODE_RX_OVERRUN) {
                msg.flags.overrun = 1;
        }

        // copy out message
        uint32_t dataIn = NFLEXCANb_MBn_WORD0(flexcanBase, buffer);
        msg.buf[3] = dataIn;
        dataIn >>= 8;
        msg.buf[2] = dataIn;
        dataIn >>= 8;
        msg.buf[1] = dataIn;
        dataIn >>= 8;
        msg.buf[0] = dataIn;

        if(msg.len > 4) {
                dataIn = NFLEXCANb_MBn_WORD1(flexcanBase, buffer);
                msg.buf[7] = dataIn;
                dataIn >>= 8;
                msg.buf[6] = dataIn;
                dataIn >>= 8;
                msg.buf[5] = dataIn;
                dataIn >>= 8;
                msg.buf[4] = dataIn;
        }

        for(uint32_t loop = msg.len; loop < 8; loop++) {
                msg.buf[loop] = 0;
        }
}

/*
 * \brief Initialize the specified ring buffer.
 *
 * \param ring - ring buffer to initialize.
 * \param buffer - buffer to use for storage.
 * \param size - size of the buffer in bytes.
 *
 * \retval None.
 *
 */

void FlexCAN::initRingBuffer(CAN_frame_ringbuffer_t &ring, volatile CAN_message_t *buffer, uint32_t size) {
        ring.buffer = buffer;
        ring.size = size;
        ring.head = 0;
        ring.tail = 0;
}

/*
 * \brief Add a CAN message to the specified ring buffer.
 *
 * \param ring - ring buffer to use.
 * \param msg - message structure to add.
 *
 * \retval true if added, false if the ring is full.
 *
 */

bool FlexCAN::addToRingBuffer(CAN_frame_ringbuffer_t &ring, const CAN_message_t &msg) {
        uint16_t nextEntry;

        nextEntry = (ring.head + 1) % ring.size;

        /* check if the ring buffer is full */
        if(nextEntry == ring.tail) {
                return (false);
        }

        /* add the element to the ring */
        memcpy((void *)&ring.buffer[ring.head], (void *)&msg, sizeof (CAN_message_t));

        /* bump the head to point to the next free entry */
        ring.head = nextEntry;

        return (true);
}

/*
 * \brief Add a CAN message to the specified ring buffer.
 *
 * \param ring - ring buffer to use.
 * \param msg - message structure to add.
 *
 * \retval true if added, false if the ring is full.
 *
 */

bool FlexCAN::coalesceToRingBuffer(CAN_frame_ringbuffer_t &ring, const CAN_message_t &msg) {
        volatile CAN_message_t *last=&ring.buffer[(ring.head + (ring.size - 1)) % ring.size];
        uint16_t newsize = last->len + msg.len;

        bool toobig = msg.len > 7 || newsize > 8;
        if(isRingBufferEmpty(ring) == true || toobig || last->id != msg.id) {
                // too big, empty buffer, or id mismatch, just add as normal
                return addToRingBuffer(ring, msg);
        }
        // we can coalesce the last message with this one.
        uint16_t i = last->len;
        volatile uint8_t *p = &(last->buf[i]);
        const uint8_t *q = msg.buf;
        memcpy((void *)p, (void *)q, msg.len);
        // set new size
        last->len = newsize;
        return (true);
}

/*
 * \brief Remove a CAN message from the specified ring buffer.
 *
 * \param ring - ring buffer to use.
 * \param msg - message structure to fill in.
 *
 * \retval true if a message was removed, false if the ring is empty.
 *
 */

bool FlexCAN::removeFromRingBuffer(CAN_frame_ringbuffer_t &ring, CAN_message_t &msg) {

        /* check if the ring buffer has data available */
        if(isRingBufferEmpty(ring) == true) {
                return (false);
        }

        /* copy the message */
        memcpy((void *)&msg, (void *)&ring.buffer[ring.tail], sizeof (CAN_message_t));

        /* bump the tail pointer */
        ring.tail = (ring.tail + 1) % ring.size;

        return (true);
}

/*
 * \brief Check if the specified ring buffer is empty.
 *
 * \param ring - ring buffer to use.
 *
 * \retval true if the ring contains data, false if the ring is empty.
 *
 */

bool FlexCAN::isRingBufferEmpty(CAN_frame_ringbuffer_t &ring) {
        if(ring.head == ring.tail) {
                return (true);
        }

        return (false);
}

/*
 * \brief Count the number of entries in the specified ring buffer.
 *
 * \param ring - ring buffer to use.
 *
 * \retval a count of the number of elements in the ring buffer.
 *
 */

uint32_t FlexCAN::ringBufferCount(CAN_frame_ringbuffer_t &ring) {
        int32_t entries;

        entries = ring.head - ring.tail;

        if(entries < 0) {
                entries += ring.size;
        }

        return ((uint32_t)entries);
}

uint32_t FlexCAN::ringBufferFree(CAN_frame_ringbuffer_t &ring) {
        int32_t entries;

        entries = ring.head - ring.tail;

        if(entries < 0) {
                entries += ring.size;
        }
        // because of how the ring buffers work, we have to subtract 1, else we never reach zero.
        entries = ring.size - entries - 1;
        return ((uint32_t)entries);
}


/*
 * \brief Interrupt service routine for the FlexCAN class message events.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void FlexCAN::message_isr(void) {
        uint32_t status = NFLEXCANb_IFLAG1(flexcanBase);
        uint8_t controller = 0;
        uint32_t i;
        CAN_message_t msg;
        bool handledFrame;
        CANListener *thisListener;
        CAN_frame_ringbuffer_t *pRing;

#if defined(COLLECT_CAN_STATS)
        uint32_t rxEntries;
#endif

#if defined (INCLUDE_FLEXCAN_CAN1)
        // determine which controller we're servicing
        if(flexcanBase == NFLEXCAN1_BASE)
                controller = 1;
#endif

        // a message either came in or was freshly sent. Figure out which and act accordingly.
        for(i = 0; i < getNumMailBoxes(); i++) {

                // skip mailboxes that haven't triggered an interrupt
                if((status & (1UL << i)) == 0) {
                        continue;
                }

                // examine the reason the mailbox interrupted us
                uint32_t code = NFLEXCAN_get_code(NFLEXCANb_MBn_CS(flexcanBase, i));

                switch(code) {

                        case NFLEXCAN_MB_CODE_RX_FULL: // rx full, Copy the frame to RX buffer
                        case NFLEXCAN_MB_CODE_RX_OVERRUN: // rx overrun. Incomming frame overwrote existing frame.
                                readRxRegisters(msg, i);
                                handledFrame = false;

#if defined(COLLECT_CAN_STATS)
                                // track message use count if collecting statistics
                                if(stats.enabled == true) {
                                        stats.mb[i].refCount++;

                                        if(msg.flags.overrun) {
                                                stats.mb[i].overrunCount++;
                                        }
                                }
#endif

                                // First, try and handle via callback. If callback fails then buffer the frame.
                                for(uint32_t listenerPos = 0; listenerPos < SIZE_LISTENERS; listenerPos++) {
                                        thisListener = listener[listenerPos];

                                        // process active listeners
                                        if(thisListener != NULL) {

                                                // call the handler if it's active for this mailbox

                                                if(thisListener->callbacksActive & (1UL << i)) {
                                                        handledFrame |= thisListener->frameHandler(msg, i, controller);
                                                } else if(thisListener->callbacksActive & (1UL << 31)) {
                                                        handledFrame |= thisListener->frameHandler(msg, -1, controller);
                                                }
                                        }
                                }

                                // if no objects caught this frame then queue it in the ring buffer
                                if(handledFrame == false) {
                                        if(addToRingBuffer(rxRing, msg) != true) {
                                                // ring buffer is full, track it

                                                dbg_println("Receiver buffer overrun!");

#if defined(COLLECT_CAN_STATS)
                                                if(stats.enabled == true) {
                                                        stats.ringRxFramesLost++;
                                                }
#endif
                                        }
                                }

#if defined(COLLECT_CAN_STATS)
                                if(stats.enabled == true) {
                                        // track the high water mark for the receive ring buffer
                                        rxEntries = ringBufferCount(rxRing);
                                        if(stats.ringRxHighWater < rxEntries) {
                                                stats.ringRxHighWater = rxEntries;
                                        }
                                }
#endif

                                // it seems filtering works by matching against the ID stored in the mailbox
                                // so after a frame comes in we've got to refresh the ID field to be the filter ID and not the ID
                                // that just came in.
                                if(MBFilters[i].flags.extended) {
                                        NFLEXCANb_MBn_ID(flexcanBase, i) = (MBFilters[i].id & NFLEXCAN_MB_ID_EXT_MASK);
                                } else {
                                        NFLEXCANb_MBn_ID(flexcanBase, i) = NFLEXCAN_MB_ID_IDSTD(MBFilters[i].id);
                                }
                                break;

                        case NFLEXCAN_MB_CODE_TX_INACTIVE: // TX inactive. Just chillin' waiting for a message to send. Let's see if we've got one.
                                // if there is a frame in the queue then send it
                                pRing = (usesGlobalTxRing(i) ? &txRing : txRings[i]);

                                if(isRingBufferEmpty(*pRing) == false) {
                                        if(removeFromRingBuffer(*pRing, msg) == true) {
                                                writeTxRegisters(msg, i);
                                        }
                                } else {
                                        for(uint32_t listenerPos = 0; listenerPos < SIZE_LISTENERS; listenerPos++) {
                                                thisListener = listener[listenerPos];

                                                // process active listeners
                                                if(thisListener != NULL) {
                                                        if(thisListener->callbacksActive & (1UL << i | 1UL << 31)) {
                                                                thisListener->txHandler(i, controller);
                                                        }
                                                }
                                        }
                                }

                                break;

                                // currently unhandled events
                        case NFLEXCAN_MB_CODE_RX_INACTIVE: // inactive Receive box. Must be a false alarm!?
                        case NFLEXCAN_MB_CODE_RX_BUSY: // mailbox is busy. Don't touch it.
                        case NFLEXCAN_MB_CODE_RX_EMPTY: // rx empty already. Why did it interrupt then?
                        case NFLEXCAN_MB_CODE_TX_ABORT: // TX being aborted.
                        case NFLEXCAN_MB_CODE_TX_RESPONSE: // remote request response (deprecated)
                        case NFLEXCAN_MB_CODE_TX_ONCE: // TX mailbox is full and will be sent as soon as possible
                        case NFLEXCAN_MB_CODE_TX_RESPONSE_TEMPO: // remote request junk again. Go away.
                                break;

                        default:
                                break;
                }
        }

        // writing the flag value back to itself clears all flags
        NFLEXCANb_IFLAG1(flexcanBase) = status;
}

/*
 * \brief Attach an object to the listening list.
 *
 * \param listener - pointer to listening object
 *
 * \retval true if listener added to list, false if the list is full.
 *
 */

bool FlexCAN::attachObj(CANListener *listener) {
        uint32_t i;

        for(i = 0; i < SIZE_LISTENERS; i++) {
                if(this->listener[i] == NULL) {
                        this->listener[i] = listener;
                        listener->callbacksActive = 0;
                        return true;
                }
        }

        return false;
}

/*
 * \brief Detatch an object from the listening list.
 *
 * \param listener - pointer to listening object
 *
 * \retval true if listener removed from list, false if object not found.
 *
 */

bool FlexCAN::detachObj(CANListener *listener) {
        uint32_t i;

        for(i = 0; i < SIZE_LISTENERS; i++) {
                if(this->listener[i] == listener) {
                        this->listener[i] = NULL;

                        return true;
                }
        }
        return false;
}

void FlexCAN::bus_off_isr(void) {
}

/*
 * \brief Interrupt service routine for FlexCAN class device errors.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void FlexCAN::error_isr(void) {
        static uint32_t status __attribute__((used)) = NFLEXCANb_ESR1(flexcanBase);
        // Clear the error(s)
        NFLEXCANb_ESR1(flexcanBase) = NFLEXCAN_ESR_STF_ERR | NFLEXCAN_ESR_FRM_ERR | NFLEXCAN_ESR_CRC_ERR | NFLEXCAN_ESR_ACK_ERR | NFLEXCAN_ESR_BIT0_ERR | NFLEXCAN_ESR_BIT1_ERR | NFLEXCAN_ESR_ERR_INT;
        // to-do callback to deal with it
}

/*
 * \brief Interrupt service routine for the FlexCAN class transmit warnings.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void FlexCAN::tx_warn_isr(void) {
        // to-do callback to deal with it
}

/*
 * \brief Interrupt service routine for the FlexCAN class receive warnings.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void FlexCAN::rx_warn_isr(void) {
        // to-do callback to deal with it
}

/*
 * \brief Interrupt service routine for the FlexCAN class device wakeup.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void FlexCAN::wakeup_isr(void) {
}

/*
 * \brief Interrupt handler for FlexCAN can0 message events.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void can0_message_isr(void) {
        Can0.message_isr();
}

/*
 * \brief Interrupt handler for FlexCAN can0 bus off event.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void can0_bus_off_isr(void) {
        Can0.bus_off_isr();
}

/*
 * \brief Interrupt handler for FlexCAN can0 error events.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void can0_error_isr(void) {
        Can0.error_isr();
}

/*
 * \brief Interrupt handler for FlexCAN can0 transmit warning event.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void can0_tx_warn_isr(void) {
        Can0.tx_warn_isr();
}

/*
 * \brief Interrupt handler for FlexCAN can0 receive warning event.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void can0_rx_warn_isr(void) {
        Can0.rx_warn_isr();
}

/*
 * \brief Interrupt handler for FlexCAN can0 device wakeup event.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void can0_wakeup_isr(void) {
        Can0.wakeup_isr();
}

#if defined(INCLUDE_FLEXCAN_CAN1)

/*
 * \brief Interrupt handler for FlexCAN can1 message events.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void can1_message_isr(void) {
        Can1.message_isr();
}

/*
 * \brief Interrupt handler for FlexCAN can1 bus off event.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void can1_bus_off_isr(void) {
        Can1.bus_off_isr();
}

/*
 * \brief Interrupt handler for FlexCAN can1 error events.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void can1_error_isr(void) {
        Can1.error_isr();
}

/*
 * \brief Interrupt handler for FlexCAN can1 transmit warning event.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void can1_tx_warn_isr(void) {
        Can1.tx_warn_isr();
}

/*
 * \brief Interrupt handler for FlexCAN can1 receive warning event.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void can1_rx_warn_isr(void) {
        Can1.rx_warn_isr();
}

/*
 * \brief Interrupt handler for FlexCAN can1 device wakeup event.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void can1_wakeup_isr(void) {
        Can1.wakeup_isr();
}

#endif /* INCLUDE_FLEXCAN_CAN1 */

/*
 * \brief CANListener constructor
 *
 * \param None.
 *
 * \retval None.
 *
 */

CANListener::CANListener() {
        // none. Bitfield were bits 0-15 are the mailboxes and bit 31 is the general callback
        callbacksActive = 0;
}

/*
 * \brief Default CAN received frame handler.
 *
 * \param frame - CAN frame to process.
 * \param mailbox - mailbox number frame arrived at.
 * \param controller - controller number frame arrived from.
 *
 * \retval true if frame was handled, false otherwise.
 *
 */

bool CANListener::frameHandler(CAN_message_t &/*frame*/, int /*mailbox*/, uint8_t /*controller*/) {
        /* default implementation that doesn't handle frames */
        return (false);
}

/*
 * \brief Default CAN transmission completed handler.
 *
 * \param mailbox - transmit mailbox that is now available.
 * \param controller - controller number.
 *
 */

void CANListener::txHandler(int /*mailbox*/, uint8_t /*controller*/) {

}

/*
 * \brief Indicate mailbox has an active callback.
 *
 * \param mailBox - mailbox number.
 *
 * \retval None.
 *
 */

void CANListener::attachMBHandler(uint8_t mailBox) {
        if((mailBox < NUM_MAILBOXES)) {
                callbacksActive |= (1L << mailBox);
        }
}

/*
 * \brief Clear callback indicator for a mailbox.
 *
 * \param mailBox - mailbox number.
 *
 * \retval None.
 *
 */

void CANListener::detachMBHandler(uint8_t mailBox) {
        if((mailBox < NUM_MAILBOXES)) {
                callbacksActive &= ~(1UL << mailBox);
        }
}

/*
 * \brief Set general purpose callback indicator.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void CANListener::attachGeneralHandler(void) {
        callbacksActive |= (1UL << 31);
}

/*
 * \brief Clear general purpose callback indicator.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void CANListener::detachGeneralHandler(void) {
        callbacksActive &= ~(1UL << 31);
}

// TO-DO: extend the possibility for console to non-KE1xF
#if defined(__MKE16F512VLH16__) || defined(__MKE16F256VLH16__)
extern "C" {

#if CAN_ADDRESS_IS_A_VARIABLE
        uint32_t MY_CAN_ADDRESS;

        void can0_setaddress(uint32_t addr) {
                MY_CAN_ADDRESS = addr & 0x000000FFu;
        }
#endif /* CAN_ADDRESS_IS_A_VARIABLE */


#if F_CAN_CONSOLE
        blconsoleListener blconsole;
#endif

#if F_CAN_CONSOLE || F_BLHOST_CAN
        int blhostMbox;
#endif

#if F_BLHOST_CAN
        blhostListener blhost;
        void start_blhost_CAN(void) {
#if CAN_ADDRESS_IS_A_VARIABLE
                MY_CAN_ADDRESS = 0x80;
#endif
                Can0.setRxBufferSize(256); // 2K DATA
                Can0.setTxBufferSize(2); // generally unused.
                Can0.setNumTxBoxes(3); // enforce order, we use the _first_ one
                blhostMbox = Can0.getFirstTxBox(); // get our private mailbox
                Can0.setMailBoxTxBufferSize(blhostMbox, 256); // eat box 1, 2K DATA
                Can0.begin(F_BLHOST_CAN);
                Can0.attachObj(&blhost);
#if F_CAN_CONSOLE
                blconsole_in.deAllocate();
                blconsole_in.init(256);
                Can0.attachObj(&blconsole);
#endif
	        CAN_filter_t allPassFilter;
        	allPassFilter.id = 0;
        	allPassFilter.ext = 1;
        	allPassFilter.rtr = 0;
                // leave the first two open to read in 11bit can
        	for(uint8_t filterNum = 2; filterNum < Can0.getNumRxBoxes(); filterNum++) {
                	Can0.setFilter(allPassFilter, filterNum);
        	}

                blhost.attachGeneralHandler();
#if F_CAN_CONSOLE
                blconsole.attachGeneralHandler();
#endif
                Can0.setListenOnly(false); // start participation.
        }
}

//
// This listener will enter boot ROM if it sees a blhost PING from CAN address 0x321
// IMPORTANT! If using filters:
// Make sure one mailbox can RX a non-extended message to address 0x321!
//

bool blhostListener::frameHandler(CAN_message_t &frame, int mailbox __attribute__((unused)), uint8_t controller __attribute__((unused))) {
        if(frame.id == 0x321 && !frame.flags.extended) {
                // 5a a6 == blhost ping
                if(frame.buf[0] == 0x5a && frame.buf[1] == 0xa6) {
                        detachGeneralHandler();
                        // run rom...
                        uint32_t runBootloaderAddress;
                        void (*runBootloader)(void * arg);
                        runBootloaderAddress = **(uint32_t **)(0x1c00001c);
                        runBootloader = (void (*)(void *))runBootloaderAddress;
                        runBootloader(NULL);
                }
                return true; // Swallow any left over/unintentional messages
        }
        return false; // Not for this handler, check next handler
}

#endif

#if F_CAN_CONSOLE
bool blconsoleListener::frameHandler(CAN_message_t &frame, int mailbox __attribute__((unused)), uint8_t controller __attribute__((unused))) {
        if(frame.flags.extended) {
                if(CAN_to_ME(frame.id)) {
                        if((frame.id & CAN_FROM_DBG) == CAN_FROM_DBG) {
                                // Stuff into emulated serial buffer, if there is room.
                                // Just like real serial, if no room, we drop the data on the floor.
                                for(int i = 0; i < frame.len; i++) {
                                        if(blconsole_in.AvailableForPut() > 0) blconsole_in.put(frame.buf[i]);
                                }
                        return true;
                        }
                }
        }
        return false; // Not for this handler, check next handler
}

size_t blconsole_::write(uint8_t c) {
        size_t rv;
        CAN_message_t msg;
        msg.id = TOCANDBUG;
        msg.flags.extended = 1;
        msg.flags.remote = 0;
        msg.flags.reserved = 0;
        msg.len = 1;
        msg.buf[0] = c;
        if(_timeout > 0) {
                unsigned long startMillis = millis();
                do {
                        rv = Can0.Cwrite(msg, blhostMbox);
                        if(rv) return rv;
                } while(millis() - startMillis < _timeout);
        } else {
                do {
                        rv = Can0.Cwrite(msg, blhostMbox);
                } while(!rv);
        }
        return rv;
}

size_t blconsole_::write(const uint8_t *buffer, size_t size) {
        int i;
        size_t pos = 0;
        size_t rv;
        CAN_message_t msg;
        msg.id = TOCANDBUG;
        msg.flags.extended = 1;
        msg.flags.remote = 0;
        msg.flags.reserved = 0;
        unsigned long startMillis = millis();
        if(_timeout > 0) {
                while((size > 0) && (millis() - startMillis < _timeout)) {
                        i = size;
                        if(i > 8) {
                                i = 8;
                        }
                        msg.len = i;
                        size -= i;
                        for(int j = 0; j < i; j++) {
                                msg.buf[j] = buffer[pos+j];
                        }
                        startMillis = millis();
                        do {
                                rv = Can0.Cwrite(msg, blhostMbox);
                                if(rv) {
                                        pos += i;
                                        break;
                                }
                        } while(millis() - startMillis < _timeout);
                }
        } else {
                while(size > 0) {
                        i = size;
                        if(i > 8) {
                                i = 8;
                        }
                        msg.len = i;
                        size -= i;
                        for(int j = 0; j < i; j++) {
                                msg.buf[j] = buffer[pos+j];
                        }
                        do {
                                rv = Can0.Cwrite(msg, blhostMbox);
                                if(rv) {
                                        pos += i;
                                        break;
                                }
                        } while(!rv);
                }
        }
        return pos;
}

extern "C" {
#if !F_BLHOST_CAN
        void start_console_CAN(void) {
#if CAN_ADDRESS_IS_A_VARIABLE
                MY_CAN_ADDRESS = 0x80;
#endif
                Can0.setTxBufferSize(128);
                Can0.setNumTxBoxes(3); // enforce order, we use the _first_ one
                blhostMbox = Can0.getFirstTxBox(); // get our private mailbox
                Can0.setMailBoxTxBufferSize(blhostMbox, 512); // eat box 1, buffering 6K-8K
                Can0.begin(F_CAN_CONSOLE);
                blconsole_in.deAllocate();
                blconsole_in.init(256);
                Can0.attachObj(&blconsole);
                blconsole.attachGeneralHandler();
	        CAN_filter_t allPassFilter;
        	allPassFilter.id = 0;
        	allPassFilter.ext = 1;
        	allPassFilter.rtr = 0;
        	for(uint8_t filterNum = 2; filterNum < Can0.getNumRxBoxes(); filterNum++) {
                	Can0.setFilter(allPassFilter, filterNum);
        	}

                blhost.attachGeneralHandler();
                Can0.setListenOnly(false); // start participation.
        }
#endif
        // for printf
        int _write(int fd, const char *ptr, int len) {
                CAN_message_t msg;
                int i;
                int pos = 0;
                bool skip = false;

                msg.id = TOCANDBUG;
                msg.flags.extended = 1;
                msg.flags.remote = 0;
                msg.flags.reserved = 0;

                while(len) {
                        i = len;
                        if(i > 8) {
                                i = 8;
                        }
                        msg.len = i;
                        for(int j = 0; j < i; j++) {
                                msg.buf[j] = ptr[pos];
                                pos++;
                        }
                        if(!skip) {
                                int rv = Can0.write(msg, blhostMbox);
                                if(!rv) {
                                        delay(1); // delay a bit if buffer was full.
                                        rv = Can0.write(msg, blhostMbox);
                                        if(!rv) skip = true; // skip the remainder, we're stuck.
                                }
                        }
                        len -= i;
                }
                return pos;
        }

        // For now, we can not get input from CAN via stdio.
        int _read(int fd, char *ptr, int len) {
                return 0;
        }

#include <sys/stat.h>

        // for printf
        int _fstat(int fd, struct stat *st) {
                memset(st, 0, sizeof (*st));
                st->st_mode = S_IFCHR;
                st->st_blksize = 1024;
                return 0;
        }

        // for printf
        int _isatty(int fd) {
                return (fd < 3) ? 1 : 0;
        }
}

#endif

#endif
