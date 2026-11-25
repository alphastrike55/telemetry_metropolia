/*
 * can.h
 *
 *  Created on: Mar 1, 2026
 *      Author: glebt
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"

typedef struct{
	uint16_t Id;
	uint8_t DLC;
	uint8_t Bytes[8];
    uint8_t CanNbr;
}CAN_Message;

void CanSend(uint8_t *TxData, uint32_t identifier, uint8_t CanBusNbr );

extern CAN_Message MessageBuffer[256];
extern volatile uint8_t MessageBufferIndex;

#define XBEE_LINE_MAX 160

/* Turns a received CAN message into one line for the XBee:
 *   HPF26/Telemetry/Can_<bus> {"seq":..,"ts":..,"frames":[{"id":..,"data":".."}]}\n
 * The XBee publishes the JSON to the topic at the start of the line.
 * Returns the line length, or 0 if this message is not sent (not in the list or too soon). */
int CanMessageToXbee(const CAN_Message *msg, char *out);

#endif /* INC_CAN_H_ */
