/*
 * test_frames.h
 *
 * Test generator: sample CAN frames of an electric FS car, sent in random
 * order over UART to the XBee as ready-made server batches.
 *
 * One UART line = one MQTT message:
 *   <topic> <json>\n
 *   HPF26/Telemetry/Can_1 {"seq":0,"ts":1795219202000,"frames":[{"id":160,"data":"E02E3A076E15035A"}]}
 */

#ifndef INC_TEST_FRAMES_H_
#define INC_TEST_FRAMES_H_

#include "main.h"

/* Call once at startup. Any non-zero seed; different seeds give different orders. */
void TestFrames_Init(uint32_t seed);

/* Builds one batch (random bus, 1..TEST_MAX_FRAMES_PER_BATCH random frames of that bus)
 * and sends it as one text line over the given UART. Blocking, ~40 ms at 115200 baud.
 * The bus sequence number is incremented only after a successful send. */
HAL_StatusTypeDef TestFrames_SendRandomBatch(UART_HandleTypeDef *huart);

#endif /* INC_TEST_FRAMES_H_ */
