/*
 * can.c
 *
 *  Created on: Mar 1, 2026
 *      Author: glebt
 */

#include <stdio.h>
#include "can.h"

extern FDCAN_HandleTypeDef hfdcan2;
//extern FDCAN_HandleTypeDef hfdcan3;

extern UART_HandleTypeDef huart5;

extern FDCAN_TxHeaderTypeDef TxHeader;

extern FDCAN_RxHeaderTypeDef RxHeader;

uint8_t CAN_RxData[8];

CAN_Message MessageBuffer[256];
volatile uint8_t MessageBufferIndex = 0;


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retreive Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, CAN_RxData) != HAL_OK)
    {
    /* Reception Error */
    Error_Handler();
    }
    else{

            MessageBuffer[MessageBufferIndex].Id = RxHeader.Identifier;
            MessageBuffer[MessageBufferIndex].DLC = RxHeader.DataLength;
            for (int i = 0; i < RxHeader.DataLength; i++)
            {
                MessageBuffer[MessageBufferIndex].Bytes[i] = CAN_RxData[i];
            }
            MessageBuffer[MessageBufferIndex].CanNbr = (hfdcan->Instance == FDCAN2) ? 1 : 2;
            MessageBufferIndex++;

    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }
  }
}

void CanSend(uint8_t *TxData, uint32_t identifier, uint8_t CanBusNbr){

	TxHeader.Identifier = identifier;

    switch (CanBusNbr)
    {
        case 1:
            while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) != 0 && HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData) != HAL_OK){
            Error_Handler();
            }
            break;
//        case 2:
//            while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) != 0 && HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData) != HAL_OK){
//            Error_Handler();
//            }
//            break;
        default:
            break;
    }
}


/* ---------------- CAN -> XBee ---------------- */

#define TOPIC_PREFIX "HPF26/Telemetry/Can_"

/* No real clock on the board yet, so ts = this start time + uptime.
 * 1795293000 = 2026-11-21 20:30:00 UTC */
#define TS_BASE_S 1795293000UL

/* Messages we send to the server, taken from can1_HPF24.dbc and can2-HPF24.dbc.
 * period_ms limits how often one ID goes out, the bus has way more traffic
 * than UART + LTE can carry. */
typedef struct {
    uint8_t  bus;
    uint16_t id;
    uint16_t period_ms;
} fwd_msg_t;

static const fwd_msg_t fwd[] = {
    /* CAN1, inverters (node 6 and node 14) */
    { 1,  390, 1000 },  /* TxPDO_DeviceActual_6      DC link V, ambient temp, DC power */
    { 1,  398, 1000 },  /* TxPDO_DeviceActual_14 */
    { 1,  582,  500 },  /* TxPDO_MotorAActual1_6     speed, torque, current */
    { 1,  838,  500 },  /* TxPDO_MotorBActual1_6 */
    { 1,  590,  500 },  /* TxPDO_MotorAActual1_14 */
    { 1,  846,  500 },  /* TxPDO_MotorBActual1_14 */
    { 1,  646, 2000 },  /* TxPDO_MotorAActual2_6     motor temp, power, voltage, module temp */
    { 1,  902, 2000 },  /* TxPDO_MotorBActual2_6 */
    { 1,  654, 2000 },  /* TxPDO_MotorAActual2_14 */
    { 1,  910, 2000 },  /* TxPDO_MotorBActual2_14 */
    { 1,  454, 1000 },  /* TxPDO_InverterAStatus_6   status word + error bits */
    { 1,  710, 1000 },  /* TxPDO_InverterBStatus_6 */
    { 1,  462, 1000 },  /* TxPDO_InverterAStatus_14 */
    { 1,  718, 1000 },  /* TxPDO_InverterBStatus_14 */

    /* CAN1, IVT and AMS */
    { 1, 1313,  200 },  /* IVT_Msg_Result_I          battery current, mA */
    { 1, 1314, 1000 },  /* IVT_Msg_Result_U1 */
    { 1,  151, 1000 },  /* SOC */

    /* CAN2, analog nodes */
    { 2,    1,  500 },  /* APPS1 */
    { 2,    2, 1000 },  /* APPS2 */
    { 2,    3,  500 },  /* BPPS */
    { 2,    4, 1000 },  /* BrakeFront */
    { 2,    5, 1000 },  /* BrakeRear */
    { 2,    6,  500 },  /* Steering_Angle */
};

#define FWD_COUNT (sizeof(fwd) / sizeof(fwd[0]))

static uint32_t fwd_last[FWD_COUNT];   /* when each ID was last sent */

/* ECU state messages on CAN2 (ID 160..187) have no signals, the ID itself
 * is the event, so all of them go through, max once per 500 ms each. */
#define ECU_STATE_FIRST 160
#define ECU_STATE_LAST  187

static uint32_t ecu_state_last[ECU_STATE_LAST - ECU_STATE_FIRST + 1];

static int time_to_send(uint32_t *last, uint32_t period, uint32_t now)
{
    if (*last != 0 && now - *last < period)
        return 0;
    *last = now;
    return 1;
}

static int should_send(uint8_t bus, uint16_t id, uint32_t now)
{
    if (bus == 2 && id >= ECU_STATE_FIRST && id <= ECU_STATE_LAST)
        return time_to_send(&ecu_state_last[id - ECU_STATE_FIRST], 500, now);

    for (unsigned i = 0; i < FWD_COUNT; i++) {
        if (fwd[i].bus == bus && fwd[i].id == id)
            return time_to_send(&fwd_last[i], fwd[i].period_ms, now);
    }
    return 0;
}

int CanMessageToXbee(const CAN_Message *msg, char *out)
{
    static const char hex[] = "0123456789ABCDEF";
    static uint32_t seq[3];   /* seq[1] for CAN1, seq[2] for CAN2 */
    uint32_t now = HAL_GetTick();

    if (!should_send(msg->CanNbr, msg->Id, now))
        return 0;

    int n = sprintf(out, TOPIC_PREFIX "%u {\"seq\":%lu,\"ts\":%lu%03lu,\"frames\":[{\"id\":%u,\"data\":\"",
                    msg->CanNbr,
                    (unsigned long)seq[msg->CanNbr]++,
                    TS_BASE_S + now / 1000, (unsigned long)(now % 1000),
                    msg->Id);

    for (uint8_t i = 0; i < msg->DLC; i++) {
        out[n++] = hex[msg->Bytes[i] >> 4];
        out[n++] = hex[msg->Bytes[i] & 0x0F];
    }

    n += sprintf(out + n, "\"}]}\n");
    return n;
}
