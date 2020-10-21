#pragma once

#include "global_type.h"

#define RX_CAN_ID ((CAN_ID_TSMR<<16) + CAN_ID_RPI)
#define CAN_ID_MASK (0x000F00FF)

void init_can_link(global_data *pdata);
int process_can_rx(global_data *pdata, CanardFrame *preceived_frame);
int decode_can_rx(global_data *pdata, CanardTransfer *ptransfer);
int tx_feed_back(global_data *pdata);
