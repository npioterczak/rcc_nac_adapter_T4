//
// Created by norbert on 12.08.22.
//
#include <FlexCAN_T4.h>

#ifndef PEUGEOT_CAN_H
#define PEUGEOT_CAN_H

class Peugeot_CAN {
public:
    static void initPeugeotCan();
    static void canBusToNac(const CAN_message_t &msg);
    static void nacToCanBus(const CAN_message_t &msg);
    static void printPacket(const CAN_message_t &msg);;

private:
    static int daysSinceYearStartFct();
    static void sendPOPup(bool present, int id, byte priority, byte parameters);

};

#endif //PEUGEOT_CAN_H
