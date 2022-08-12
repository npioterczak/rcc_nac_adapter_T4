#include <Arduino.h>
#include <Peugeot_CAN.cpp>

extern FlexCAN_T4 <CAN1, RX_SIZE_256, TX_SIZE_16> can2004;
extern FlexCAN_T4 <CAN2, RX_SIZE_256, TX_SIZE_16> can2010;
CAN_message_t msg;

void setup() {
// init can from car entertainment to nac/rcc
    
    // can2010.enableFIFO();
// init setup
    Peugeot_CAN::initPeugeotCan();

}

void loop() {
    if ( can2004.read(msg) ) {
        Peugeot_CAN::canBusToNac(msg);
    }
    if ( can2010.read(msg) ) {
        Peugeot_CAN::nacToCanBus(msg);
    }
    // can2004.mailboxStatus();
}

