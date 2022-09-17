/*
 * Sniff the CAN bus
 */
#include <Arduino.h>
#include <NetFlexCAN.h>

class ExampleClass : public CANListener {
public:
        void printFrame(CAN_message_t &frame, int mailbox);
        bool frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller); //overrides the parent version so we can actually do something
};

void ExampleClass::printFrame(CAN_message_t &frame, int mailbox) {
        Serial.print("FRAME is ");
        if(frame.flags.extended) Serial.print("Extended");
        else Serial.print("Classic");
        Serial.print(" ID: 0x");
        Serial.print((frame.id & 0xf0000000u) >> 28, HEX);
        Serial.print((frame.id & 0xf000000u) >> 24, HEX);
        Serial.print((frame.id & 0xf00000u) >> 20, HEX);
        Serial.print((frame.id & 0xf0000u) >> 16, HEX);
        Serial.print((frame.id & 0xf000u) >> 12, HEX);
        Serial.print((frame.id & 0xf00u) >> 8, HEX);
        Serial.print((frame.id & 0xf0u) >> 4, HEX);
        Serial.print(frame.id & 0x0fu, HEX);
        Serial.print(" Data:");
        for(int c = 0; c < frame.len; c++) {
                Serial.print(" 0x");
                Serial.print((frame.buf[c] & 0xf0) >> 4, HEX);
                Serial.print(frame.buf[c] & 0xf, HEX);
        }
        Serial.println("");
}

bool ExampleClass::frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller) {
        if(Serial) printFrame(frame, mailbox);
        return true;
}

ExampleClass exampleClass;

// -------------------------------------------------------------

void setup(void) {
        while(!Serial) delay(1);
        delay(250);
        Serial.begin(115200);
        Serial.println("");
        Serial.println("");
        Serial.println("");
        Serial.println("The following keys set the speed:");
        Serial.println("1 50000  2 100000  3 125000  4 250000  5 500000  6 1000000");
        Serial.println("");
        Serial.println("");
        Serial.println("");
        Serial.flush();
        Can0.begin(500000);
        Can0.setListenOnly(true);
        Can0.attachObj(&exampleClass);
        exampleClass.attachGeneralHandler();
}


// -------------------------------------------------------------

// keys set speeds. 
// key  Speed
// 1    50000
// 2    100000
// 3    125000
// 4    250000
// 5    500000
// 6    1000000
// 
int ch = 0;
void loop(void) {
        if(Serial) {
                if(Serial.available()) {
                        int c = Serial.read();
                        switch(c) {
                                case '1':
                                        Serial.println("");
                                        Serial.println("Can rate: 50000");
                                         Serial.println("");
                                        Serial.flush();
                                        Can0.ChangeBaudRate(50000);
                                        break;
                                case '2':
                                        Serial.println("");
                                        Serial.println("Can rate: 100000");
                                         Serial.println("");
                                        Serial.flush();
                                        Can0.ChangeBaudRate(100000);
                                        break;
                                case '3':
                                        Serial.println("");
                                        Serial.println("Can rate: 125000");
                                         Serial.println("");
                                        Serial.flush();
                                        Can0.ChangeBaudRate(125000);
                                        break;
                                case '4':
                                        Serial.println("");
                                        Serial.println("Can rate: 250000");
                                         Serial.println("");
                                        Serial.flush();
                                        Can0.ChangeBaudRate(250000);
                                        break;
                                case '5':
                                        Serial.println("");
                                        Serial.println("Can rate: 500000");
                                         Serial.println("");
                                        Serial.flush();
                                        Can0.ChangeBaudRate(500000);
                                        break;
                                case '6':
                                        Serial.println("");
                                        Serial.println("Can rate: 1000000");
                                         Serial.println("");
                                        Serial.flush();
                                        Can0.ChangeBaudRate(1000000);
                                        break;
                                default:
                                        // nop
                                        break;
                        }
                }
        }
}

