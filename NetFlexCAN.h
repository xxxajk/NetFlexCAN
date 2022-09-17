// -------------------------------------------------------------
// a simple Arduino Teensy 3.1/3.2/3.6 CAN driver
// by teachop
// dual CAN support for MK66FX1M0 by Pawelsky
//
#ifndef __FLEXCAN_H__
#define __FLEXCAN_H__

#include <Arduino.h>
#include "kinetis_flexcan.h"
#if F_CAN_CONSOLE
#include <UHS_ByteBuffer.h>
extern UHS_ByteBuffer blconsole_in;
#endif

#define FlexCAN_MAILBOX_TX_BUFFER_SUPPORT  // helper definition for handling different FlexCAN revisions
#define FlexCAN_DYNAMIC_BUFFER_SUPPORT  // helper definition for handling different FlexCAN revisions

#if !defined(SIZE_RX_BUFFER)
#define SIZE_RX_BUFFER  32 // receive incoming ring buffer size
#endif

#if !defined(SIZE_TX_BUFFER)
#define SIZE_TX_BUFFER  16 // transmit ring buffer size
#endif

#define SIZE_LISTENERS  4  // number of classes that can register as listeners on each CAN bus
#define NUM_MAILBOXES   16 // architecture specific but all Teensy 3.x boards have 16 mailboxes
#define IRQ_PRIORITY    64 // 0 = highest, 255 = lowest

#define COLLECT_CAN_STATS

// source message bitmap -- each device has an address.
// one of them is this program.
// set bit 10 to lose to 11bit can, but doesn't?
// 000 0000 0000
#define                           CAN_SRC_0 (0x00000000u) // highest source priority
#define                           CAN_SRC_1 (0x00000001u)
#define                           CAN_SRC_2 (0x00000002u)
#define                           CAN_SRC_3 (0x00000004u)
#define                           CAN_SRC_4 (0x00000008u)
#define                           CAN_SRC_5 (0x00000010u)
#define                           CAN_SRC_6 (0x00000020u)
#define                           CAN_SRC_7 (0x00000040u)
#define                           CAN_SRC_8 (0x00000080u) // lowest source priority
#define                        CAN_SRC_DBUG (0x000000FFu) // rock bottom source, only for debugger

// source message priority.
#define                        SRC_MSG_PRI0 (0x00000000u) // highest priority
#define                        SRC_MSG_PRI1 (0x00000100u)
#define                        SRC_MSG_PRI2 (0x00000200u)
#define                        SRC_MSG_PRI3 (0x00000400u)
#define                        SRC_MSG_PRI4 (0x00000800u) // lowest priority
#define                        SRC_MSG_DBUG (0x00000F00u) // rock bottom priority, only for debug message

// message types, 5 + one debug type
#define                         MSG_IS_TYP0 (0x00000000u) // highest priority
#define                         MSG_IS_TYP1 (0x00001000u)
#define                         MSG_IS_TYP2 (0x00002000u)
#define                         MSG_IS_TYP3 (0x00004000u)
#define                         MSG_IS_TYP4 (0x00008000u) // lowest priority
#define                         MSG_TYPE(x) ((uint32_t)((uint32_t) x & 0x0000000fu) << 12)
#define                         MSG_IS_DBUG MSG_TYPE(0x0000000fu) // rock bottom priority, only for debug messages

// Destination message bitmap -- each device has an address.
// one of them is this program.
#define                           CAN_DST_0 (0x00000000u) // highest destination priority
#define                           CAN_DST_1 (0x00010000u)
#define                           CAN_DST_2 (0x00020000u)
#define                           CAN_DST_3 (0x00040000u)
#define                           CAN_DST_4 (0x00080000u)
#define                           CAN_DST_5 (0x00100000u)
#define                           CAN_DST_6 (0x00200000u)
#define                           CAN_DST_7 (0x00400000u)
#define                           CAN_DST_8 (0x00800000u) // lowest destination priority
#define                        CAN_DST_DBUG (0x00FF0000u) // rock bottom destination, only to debugger

// destination message priority.
#define                        DST_MSG_PRI0 (0x00000000u) // highest priority
#define                        DST_MSG_PRI1 (0x01000000u)
#define                        DST_MSG_PRI2 (0x02000000u)
#define                        DST_MSG_PRI3 (0x04000000u)
#define                        DST_MSG_PRI4 (0x08000000u) // lowest priority
#define                        DST_MSG_DBUG (0x0F000000u) // rock bottom priority, debug only

// Message priority
#define                        DST_MSG_PRIH (0x00000000u) // highest priority
#define                        DST_MSG_PRIL (0x10000000u) // lowest priority
#define                           SRC_ID(x) ((uint32_t)((uint32_t)x & 0x000000FFu))
#define                           DST_ID(x) ((uint32_t)(((uint32_t)x & 0x000000FFu) << 16))
#define                  PAIR_ID(src, dest) (SRC_ID(src)|DST_ID(dest))
#define       CHECK_ID(FRAME_ID, src, dest) (PAIR_ID(src,dest) == (FRAME_ID & (PAIR_ID(0xFFu, 0xFFu))))

#define                   CAN_DEBUG_MESSAGE (SRC_MSG_DBUG | MSG_IS_DBUG | DST_MSG_DBUG | DST_MSG_PRIL)
#define                        CAN_FROM_DBG (CAN_SRC_DBUG | CAN_DEBUG_MESSAGE)
#define                          CAN_TO_DBG (CAN_DST_DBUG | CAN_DEBUG_MESSAGE)

typedef struct CAN_message_t {
        uint32_t id; // can identifier
        uint16_t timestamp; // FlexCAN time when message arrived

        struct {
                uint8_t extended : 1; // identifier is extended (29-bit)
                uint8_t remote : 1; // remote transmission request packet type
                uint8_t overrun : 1; // message overrun
                uint8_t reserved : 5;
        } flags;
        uint8_t len; // length of data
        uint8_t buf[8];
} CAN_message_t;

typedef struct CAN_filter_t {
        uint32_t id;

        struct {
                uint8_t extended : 1; // identifier is extended (29-bit)
                uint8_t remote : 1; // remote transmission request packet type
                uint8_t reserved : 6;
        } flags;
} CAN_filter_t;

// statistics about the CAN interface

typedef struct CAN_stats_t {
        bool enabled; // enable collecting statistics
        uint32_t ringRxMax; // number of entries in the ring buffer
        uint32_t ringRxHighWater; // maximum entries used in the ring buffer
        uint32_t ringRxFramesLost; // total number of frames lost
        uint32_t ringTxMax; // number of entries in the ring buffer
        uint32_t ringTxHighWater; // maximum entries used in the ring buffer

        struct {
                uint32_t refCount; // mailbox reference (use) count
                uint32_t overrunCount; // mailbox message overrun count
        } mb[NUM_MAILBOXES];
} CAN_stats_t;

// ring buffer data structure

typedef struct CAN_frame_ringbuffer_t {
        volatile uint16_t head;
        volatile uint16_t tail;
        uint16_t size;
        volatile CAN_message_t *buffer;
} CAN_frame_ringbuffer_t;

// for backwards compatibility with previous structure members

#define ext flags.extended
#define rtr flags.remote

class CANListener {
public:
        CANListener();

        virtual bool frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller);
        virtual void txHandler(int mailbox, uint8_t controller);

        void attachMBHandler(uint8_t mailBox);
        void detachMBHandler(uint8_t mailBox);
        void attachGeneralHandler(void);
        void detachGeneralHandler(void);

private:
        uint32_t callbacksActive; // bitfield indicating which callbacks are installed (for object oriented callbacks only)

        friend class FlexCAN; // class has to have access to the the guts of this one
};

// -------------------------------------------------------------

class FlexCAN {
private:
        uint32_t flexcanBase;
        struct CAN_filter_t MBFilters[NUM_MAILBOXES];
        static struct CAN_filter_t defaultMask;
        void mailbox_int_handler(uint8_t mb, uint32_t ul_status);
        CANListener *listener[SIZE_LISTENERS];

        CAN_frame_ringbuffer_t txRing;
        volatile CAN_message_t *tx_buffer;
        CAN_frame_ringbuffer_t rxRing;
        volatile CAN_message_t *rx_buffer;
        CAN_frame_ringbuffer_t * txRings[NUM_MAILBOXES];

        bool IrqEnabled;
        uint32_t IrqMessage;
        //uint32_t IrqWarn;
        //uint32_t IrqError;
        //uint32_t IrqWake;

        uint32_t baud_rate;

        void writeTxRegisters(const CAN_message_t &msg, uint8_t buffer);
        void readRxRegisters(CAN_message_t &msg, uint8_t buffer);

        void initRingBuffer(CAN_frame_ringbuffer_t &ring, volatile CAN_message_t *buffer, uint32_t size);
        bool addToRingBuffer(CAN_frame_ringbuffer_t &ring, const CAN_message_t &msg);
        bool coalesceToRingBuffer(CAN_frame_ringbuffer_t &ring, const CAN_message_t &msg);
        bool removeFromRingBuffer(CAN_frame_ringbuffer_t &ring, CAN_message_t &msg);
        bool isRingBufferEmpty(CAN_frame_ringbuffer_t &ring);
        uint32_t ringBufferCount(CAN_frame_ringbuffer_t &ring);
        uint32_t ringBufferFree(CAN_frame_ringbuffer_t &ring);

        void irqLock() {
                IrqEnabled = NVIC_IS_ENABLED(IrqMessage);
                NVIC_DISABLE_IRQ(IrqMessage);
        };

        void irqRelease() {
                if(IrqEnabled) NVIC_ENABLE_IRQ(IrqMessage);
        };

        void initializeBuffers();

        bool isInitialized() {
                return tx_buffer != 0;
        };
        void setPins(uint8_t txAlt, uint8_t rxAlt);
        void setBaudRate(uint32_t baud);
        void softReset();
        void halt();
        void exitHalt();
        void freeze();
        void waitFrozen();
        void waitNotFrozen();
        void waitReady();
        bool isFrozen();

        bool usesGlobalTxRing(uint8_t mbox) {
                return (mbox < getNumMailBoxes() ? txRings[mbox] == 0 : true);
        };

        bool isTxBox(uint8_t mbox) {
                return (mbox >= getFirstTxBox() && mbox < getNumMailBoxes());
        };

#ifdef COLLECT_CAN_STATS
        CAN_stats_t stats;
#endif

protected:
        uint8_t numTxMailboxes;
        uint16_t sizeRxBuffer;
        uint16_t sizeTxBuffer;

public:
        FlexCAN(uint8_t id = 0);

        // Before begin, you can define rx buffer size. Default is SIZE_RX_BUFFER. This does not have effect after begin.

        void setRxBufferSize(uint16_t size) {
                if(!isInitialized()) sizeRxBuffer = size;
        };

        // Before begin, you can define global tx buffer size. Default is SIZE_TX_BUFFER. This does not have effect after begin.

        void setTxBufferSize(uint16_t size) {
                if(!isInitialized()) sizeTxBuffer = size;
        };

        // You can define mailbox specific tx buffer size. This can be defined only once per mailbox.
        // As default prioritized messages will not be buffered. If you define buffer size for mail box, the messages will be
        // buffered to own buffer, if necessary.
        void setMailBoxTxBufferSize(uint8_t mbox, uint16_t size);

        inline uint8_t getFirstTxBox() {
                return getNumMailBoxes() - numTxMailboxes;
        };

        inline uint8_t getLastTxBox() {
                return getNumMailBoxes() - 1;
        };

        inline uint8_t getNumMailBoxes() {
                return NUM_MAILBOXES;
        };

        inline uint8_t getNumRxBoxes() {
                return getNumMailBoxes() - numTxMailboxes;
        };

        void begin(uint32_t baud = 250000, const CAN_filter_t &mask = defaultMask, uint8_t txAlt = 0, uint8_t rxAlt = 0);

        void setFilter(const CAN_filter_t &filter, uint8_t n);
        bool getFilter(CAN_filter_t &filter, uint8_t n);
        void setMask(uint32_t mask, uint8_t n);
        void end(void);
        uint32_t available(void);
        int write(const CAN_message_t &msg);
        int write(const CAN_message_t &msg, uint8_t n);
        int Cwrite(const CAN_message_t &msg, uint8_t n);
        int read(CAN_message_t &msg);

        void abort(uint8_t mbox) {
                // Sloppy, but works.
                NFLEXCANb_MBn_CS(flexcanBase, mbox) = NFLEXCAN_MB_CS_CODE(NFLEXCAN_MB_CODE_TX_ABORT);
        };

        uint32_t rxBufferOverruns(void) {
                return stats.ringRxFramesLost;
        };
        void ChangeBaudRate(uint32_t baud);
#ifdef COLLECT_CAN_STATS

        void startStats(void) {
                stats.enabled = true;
        };

        void stopStats(void) {
                stats.enabled = false;
        };
        void clearStats(void);

        CAN_stats_t getStats(void) {
                return stats;
        };
#endif

        //new functionality added to header but not yet implemented. Fix me
        void setListenOnly(bool mode); //pass true to go into listen only mode, false to be in normal mode
        void setEcho(bool mode);

        bool attachObj(CANListener *listener);
        bool detachObj(CANListener *listener);

        //int watchFor(); //allow anything through
        //int watchFor(uint32_t id); //allow just this ID through (automatic determination of extended status)
        //int watchFor(uint32_t id, uint32_t mask); //allow a range of ids through
        //int watchForRange(uint32_t id1, uint32_t id2); //try to allow the range from id1 to id2 - automatically determine base ID and mask

        uint8_t setNumTxBoxes(uint8_t txboxes);
        // Obsolete, for compatibility with version provided with Teensyduino

        uint8_t setNumTXBoxes(uint8_t txboxes) {
                return setNumTxBoxes(txboxes);
        };

        void message_isr(void);
        void bus_off_isr(void);
        void error_isr(void);
        void tx_warn_isr(void);
        void rx_warn_isr(void);
        void wakeup_isr(void);
        void flush(void);
        void flush(uint8_t n);

        uint32_t get_baud(void) {
                return baud_rate;
        };
        int availableForWrite(void);
        int availableForWrite(uint8_t mbox);
};

extern FlexCAN Can0;
#if defined(__MK66FX1M0__) || defined(__IMXRT1062__)
extern FlexCAN Can1;
#if defined(__IMXRT1062__)
extern FlexCAN Can2;
#endif
#endif

#if !defined(F_BLHOST_CAN)
#define F_BLHOST_CAN 0
#endif
#if !defined(F_CAN_CONSOLE)
#define F_CAN_CONSOLE 0
#endif

// In the event we want a configurable variable from pins or flash, etc...
// Note that unspecfied is a variable, but set to 0x80 by default.
#if !defined(CAN_ADDRESS_IS_A_VARIABLE)
#if !defined(MY_CAN_ADDRESS)
#define CAN_ADDRESS_IS_A_VARIABLE 1
#else
#define CAN_ADDRESS_IS_A_VARIABLE 0
#endif
#endif

#if CAN_ADDRESS_IS_A_VARIABLE
extern "C" {
        extern uint32_t MY_CAN_ADDRESS;
        // since we are a variable, expose setting it.
        extern void can0_setaddress(uint32_t);
}
#endif

#if F_BLHOST_CAN || F_CAN_CONSOLE
#define RCV_DFLT (MY_CAN_ADDRESS << 16) // Any message for us.
#define CAN_to_ME(FRAME_ID)  ((FRAME_ID & CAN_DST_DBUG) == RCV_DFLT)

#define   TOCANDBUG (MY_CAN_ADDRESS | CAN_TO_DBG) // Debug message TO debugger

#endif /* F_BLHOST_CAN || F_CAN_CONSOLE */

#if defined(__MKE16F512VLH16__) || defined(__MKE16F256VLH16__)
#if F_BLHOST_CAN
// This listener will enter boot ROM if it sees a blhost PING from CAN address 0x321

class blhostListener : public CANListener {
public:
        bool frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller);
};

extern "C" {
        extern blhostListener blhost;
}
#endif /* F_BLHOST_CAN */

#if F_CAN_CONSOLE

class blconsoleListener : public CANListener {
public:
        bool frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller);
};

extern "C" {
        extern blconsoleListener blconsole;
        extern int blhostMbox;
        void sercanEvent() __attribute__ ((weak));
}


// Stream/print interface
#include <Stream.h>

class blconsole_ : public Stream {
public:

        virtual void begin(long) {
        };

        virtual void end() {
        };

        virtual int available() {
                return blconsole_in.getSize();
        };

        virtual int read() {
                if(blconsole_in.getSize() < 1) return -1;
                int c = blconsole_in.get();
                return c;
        };

        virtual int peek() {
                if(blconsole_in.getSize() < 1) return -1;
                int c = blconsole_in.peek(0);
                return c;
        };

        virtual void clear(void) {
                blconsole_in.clear();
        };

        virtual void flush() {
                Can0.flush(blhostMbox);
        };
        virtual size_t write(uint8_t c);
        virtual size_t write(const uint8_t *buffer, size_t size);

        virtual size_t write(unsigned long n) {
                return write((uint8_t)n);
        };

        virtual size_t write(long n) {
                return write((uint8_t)n);
        };

        virtual size_t write(unsigned int n) {
                return write((uint8_t)n);
        };

        virtual size_t write(int n) {
                return write((uint8_t)n);
        };

        virtual int availableForWrite() {
                return Can0.availableForWrite(blhostMbox);
        };
        using Print::write;

        virtual void send_now(void) {
                flush();
        };

        virtual uint32_t baud(void) {
                return Can0.get_baud();
        };

        virtual uint8_t stopbits(void) {
                return 1;
        };

        virtual uint8_t paritytype(void) {
                return 0;
        };

        virtual uint8_t numbits(void) {
                return 8;
        };

        virtual uint8_t dtr(void) {
                return 1;
        };

        virtual uint8_t rts(void) {
                return 1;
        };

        operator bool() {
                return true;
        };

        virtual size_t readBytes(char *buffer, size_t length) {
                size_t count = 0;
                size_t q;
                unsigned long startMillis = millis();
                if(_timeout > 0) {
                        do {
                                q = blconsole_in.getSize();
                                if(q > 0) {
                                        for(size_t i = 0; i < q; i++) {
                                                if(count >= length) return count;
                                                buffer[count] = read();
                                                count++;
                                        }
                                }
                                if(count >= length) return count;
                        } while(millis() - startMillis < _timeout);
                        setReadError();
                } else {
                        do {
                                q = blconsole_in.getSize();
                                if(q > 0) {
                                        for(size_t i = 0; i < q; i++) {
                                                if(count >= length) return count;
                                                buffer[count] = read();
                                                count++;
                                        }
                                }
                                if(count >= length) return count;
                        } while(count >= length);
                }
                return count;
        };
};

extern "C" {
        extern blconsole_ Sercan;
}
#endif


#endif /* __MKE16F512VLH16__ */

#endif // __FLEXCAN_H__
