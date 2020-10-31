#include "canard_link.h"
#include "can_defines.h"

#include "lowlevel/uart.h"

#include <stdlib.h>
#include <libopencm3/stm32/can.h>

//rx callback handling, don't use pdata_g outside
static global_data *pdata_g;
void can_rx_handler(uint8_t fifo, uint8_t pending, bool full, bool overrun)
{
  bool ext, rtr;
  uint8_t fmi, len = 0;
  uint16_t timestamp;
  uint32_t id;
  uint8_t data[8];

  // Receive the Great bits of the CAN, the First of their Frame,
  // Breaker of the CAN filters, Slayer of CPU time
  can_receive(CAN1,
	      0,
	      true, // release FIFO
	      &id,
	      &ext,
	      &rtr,
	      &fmi,
	      &len,
	      data,
	      &timestamp);

  if((id & CAN_ID_MASK) != RX_CAN_ID)
  {
    // All you had to do was filtering the damn trame, ST !
    return;
  }

  CanardFrame received_frame;
  received_frame.timestamp_usec = 0;
  received_frame.extended_can_id = id;
  received_frame.payload_size = len;
  received_frame.payload = data;

  int result = process_can_rx(pdata_g, &received_frame);

  // Don't care
  (void)full;
  (void)overrun;
  (void)ext;
  (void)rtr;
  (void)fmi;
  (void)timestamp;
  (void)fifo;
  (void)result;
  (void)pending;
}





static CanardRxSubscription tsmr_in_subscription;
static CanardRxSubscription tsmr_enc_l_subscription;
static CanardRxSubscription tsmr_enc_r_subscription;
static CanardRxSubscription tsmr_odom_subscription;
static CanardRxSubscription tsmr_power_subscription;
static CanardRxSubscription tsmr_t_subscription;
static CanardRxSubscription tsmr_ts_subscription;
static CanardRxSubscription tsmr_r_subscription;
static CanardRxSubscription tsmr_rs_subscription;
static CanardRxSubscription tsmr_xyt_subscription;
static CanardRxSubscription tsmr_xyt_back_subscription;

static volatile int tsmr_out_transfer_id;
static volatile int tsmr_encl_transfer_id;
static volatile int tsmr_encr_transfer_id;
static volatile int tsmr_odom_transfer_id;
static volatile int tsmr_power_transfer_id;
static volatile int tsmr_t_transfer_id;
static volatile int tsmr_ts_transfer_id;
static volatile int tsmr_r_transfer_id;
static volatile int tsmr_rs_transfer_id;
static volatile int tsmr_xyt_transfer_id;
static volatile int tsmr_xyt_back_transfer_id;


static void* memAllocate(CanardInstance* const ins, const size_t amount)
{
    (void) ins;
    //return o1heapAllocate(my_allocator, amount);
    return malloc(amount);
}

static void memFree(CanardInstance* const ins, void* const pointer)
{
    (void) ins;
    //o1heapFree(my_allocator, pointer);
    free(pointer);
}

void init_can_link(global_data *pdata)
{
  pdata_g = pdata;

  pdata->stream_enc_l = 1;
  pdata->stream_enc_r = 1;
  pdata->stream_power = 1;

  pdata->translation_to_set = 0;
  pdata->translation_speed_to_set = 0;
  pdata->rotation_to_set = 0;
  pdata->rotation_speed_to_set = 0;
  pdata->XYtheta_to_set = 0;

  tsmr_out_transfer_id = 0;
  tsmr_encl_transfer_id = 0;
  tsmr_encr_transfer_id = 0;
  tsmr_odom_transfer_id = 0;
  tsmr_power_transfer_id = 0;
  tsmr_r_transfer_id = 0;
  tsmr_rs_transfer_id = 0;
  tsmr_t_transfer_id = 0;
  tsmr_ts_transfer_id = 0;
  tsmr_xyt_transfer_id = 0;
  tsmr_xyt_back_transfer_id = 0;

  pdata->can_ins = canardInit(&memAllocate, &memFree);
  pdata->can_ins.mtu_bytes = CANARD_MTU_CAN_CLASSIC;  // Defaults to 64 (CAN FD); here we select Classic CAN.
  pdata->can_ins.node_id   = CAN_ID_TSMR;

  (void) canardRxSubscribe(&pdata->can_ins,   // Subscribe to an arbitrary service response.
                         CanardTransferKindMessage,  // Specify that we want service responses, not requests.
                         TSMR_TEXT_SET,    // The Service-ID whose responses we will receive.
                         128,   // The extent (the maximum payload size); pick a huge value if not sure.
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &tsmr_in_subscription);

  (void) canardRxSubscribe(&pdata->can_ins,
                         CanardTransferKindMessage,
                         TSMR_ENCL_SET,
                         4,
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &tsmr_enc_l_subscription);

  (void) canardRxSubscribe(&pdata->can_ins,
                         CanardTransferKindMessage,
                         TSMR_ENCR_SET,
                         4,
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &tsmr_enc_r_subscription);

  (void) canardRxSubscribe(&pdata->can_ins,
                         CanardTransferKindMessage,
                         TSMR_ODOM_SET,
                         12,
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &tsmr_odom_subscription);

  (void) canardRxSubscribe(&pdata->can_ins,
                         CanardTransferKindMessage,
                         TSMR_POWER_SET,
                         1,
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &tsmr_power_subscription);

  (void) canardRxSubscribe(&pdata->can_ins,
                         CanardTransferKindMessage,
                         TSMR_T_SET,
                         4,
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &tsmr_t_subscription);

  (void) canardRxSubscribe(&pdata->can_ins,
                        CanardTransferKindMessage,
                        TSMR_TS_SET,
                        4,
                        CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                        &tsmr_ts_subscription);

  (void) canardRxSubscribe(&pdata->can_ins,
                         CanardTransferKindMessage,
                         TSMR_R_SET,
                         4,
                         CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                         &tsmr_r_subscription);

  (void) canardRxSubscribe(&pdata->can_ins,
                        CanardTransferKindMessage,
                        TSMR_RS_SET,
                        4,
                        CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                        &tsmr_rs_subscription);

  (void) canardRxSubscribe(&pdata->can_ins,
                        CanardTransferKindMessage,
                        TSMR_XYT_SET,
                        12,
                        CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                        &tsmr_xyt_subscription);
  (void) canardRxSubscribe(&pdata->can_ins,
                        CanardTransferKindMessage,
                        TSMR_XYT_BACK_SET,
                        12,
                        CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                        &tsmr_xyt_back_subscription);
}

int canard_send_tx_queue(CanardInstance *pins)
{
  for (const CanardFrame* txf = NULL; (txf = canardTxPeek(pins)) != NULL;)  // Look at the top of the TX queue.
  {
    // Please ensure TX deadline not expired.
    // Send the frame. Redundant interfaces may be used here.

    //trying to send, -1 is sending queue full
    while(can_transmit(CAN1, txf->extended_can_id, 1, 0, txf->payload_size, (uint8_t *) txf->payload) == -1);
                                 // If the driver is busy, break and retry later.
    canardTxPop(pins);                         // Remove the frame from the queue after it's transmitted.
    pins->memory_free(pins, (CanardFrame*)txf);  // Deallocate the dynamic memory afterwards.
  }
  return 1;
}


int process_can_rx(global_data *pdata, CanardFrame *preceived_frame)
{
  CanardTransfer transfer;
  const int8_t result = canardRxAccept(&pdata->can_ins,
                                       preceived_frame,            // The CAN frame received from the bus.
                                       0,  // If the transport is not redundant, use 0.
                                       &transfer);

  if (result < 0)
  {
    //uart_send_string("Got an error\n");
  }
  else if (result == 1)
  {
    decode_can_rx(pdata, &transfer);
    //uart_send_string("Got something\n");
    pdata->can_ins.memory_free(&pdata->can_ins, (void*)transfer.payload);  // Deallocate the dynamic memory afterwards.
  }
  else
  {
    //uart_send_string("nothing\n");

  }
  return 1;
}

int decode_can_rx(global_data *pdata, CanardTransfer *ptransfer)
{
  if( ptransfer->port_id == TSMR_TEXT_SET )
  {
    for(unsigned int i=0; i<ptransfer->payload_size; i++)
    {
      char to_send[2];
      to_send[0] = ((char*)ptransfer->payload)[i];
      to_send[1] = 0;
      uart_send_string(to_send);
    }
    uart_send_string("\n");
  }
  else if( ptransfer->port_id == TSMR_ENCL_SET )
  {
    if(ptransfer->payload_size != 2)
      return 0;
    pdata->stream_enc_l = ((int16_t*)ptransfer->payload)[0];
  }
  else if( ptransfer->port_id == TSMR_ENCR_SET )
  {
    if(ptransfer->payload_size != 2)
      return 0;
    pdata->stream_enc_r = ((int16_t*)ptransfer->payload)[0];
  }
  else if( ptransfer->port_id == TSMR_POWER_SET )
  {
    if(ptransfer->payload_size != 1)
      return 0;
    pdata->stream_power = ((char*)ptransfer->payload)[0];
  }
  else if( ptransfer->port_id == TSMR_ODOM_SET )
  {
    if(ptransfer->payload_size != 12)
      return 0;
    pdata->x_set = ((float *)ptransfer->payload)[0];
    pdata->y_set = ((float *)ptransfer->payload)[1];
    pdata->theta_set = ((float *)ptransfer->payload)[2];
    pdata->odom_to_set = 1;
  }
  else if( ptransfer->port_id == TSMR_T_SET )
  {
    if(ptransfer->payload_size != 4)
      return 0;
    pdata->translation_to_set_value = ((float*)ptransfer->payload)[0];
    pdata->translation_to_set = 1;
  }
  else if( ptransfer->port_id == TSMR_TS_SET )
  {
    if(ptransfer->payload_size != 4)
      return 0;
    pdata->translation_speed_to_set_value = ((float*)ptransfer->payload)[0];
    pdata->translation_speed_to_set = 1;
  }
  else if( ptransfer->port_id == TSMR_R_SET )
  {
    if(ptransfer->payload_size != 4)
      return 0;
    pdata->rotation_to_set_value = ((float*)ptransfer->payload)[0];
    pdata->rotation_to_set = 1;
  }
  else if( ptransfer->port_id == TSMR_RS_SET )
  {
    if(ptransfer->payload_size != 4)
      return 0;
    pdata->rotation_speed_to_set_value = ((float*)ptransfer->payload)[0];
    pdata->rotation_speed_to_set = 1;
  }
  else if( ptransfer->port_id == TSMR_XYT_SET || ptransfer->port_id == TSMR_XYT_BACK_SET )
  {
    if(ptransfer->payload_size != 12)
      return 0;
    pdata->XYtheta_value_x = ((float*)ptransfer->payload)[0];
    pdata->XYtheta_value_y = ((float*)ptransfer->payload)[1];
    pdata->XYtheta_value_theta = ((float*)ptransfer->payload)[2];
    if(ptransfer->port_id == TSMR_XYT_SET )
      pdata->XYtheta_to_set = 1;
    else
      pdata->XYtheta_to_set = 2;
  }
  else
  {
    return 0;
  }
  return 1;
}


int tx_feed_back(global_data *pdata)
{
  int32_t result;

  uint16_t enc_count;

  CanardTransfer transfer = {
      .timestamp_usec = 0,      // Zero if transmission deadline is not limited.
      .priority       = CanardPriorityNominal,
      .transfer_kind  = CanardTransferKindMessage,
      .remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
  };

  if(pdata->stream_enc_l)
  {
    transfer.port_id        = TSMR_ENCL_GET;
    transfer.transfer_id    = tsmr_encl_transfer_id;
    transfer.payload_size   = 2;
    enc_count = pdata->odom.left_count;
    transfer.payload        = &enc_count;
    ++tsmr_encl_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
    result = canardTxPush(&pdata->can_ins, &transfer);
    canard_send_tx_queue(&pdata->can_ins);
  }

  if(pdata->stream_enc_r)
  {
    transfer.port_id        = TSMR_ENCR_GET;
    transfer.transfer_id    = tsmr_encr_transfer_id;
    transfer.payload_size   = 2;
    enc_count = pdata->odom.right_count;
    transfer.payload        = &enc_count;
    ++tsmr_encr_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
    result = canardTxPush(&pdata->can_ins, &transfer);
    canard_send_tx_queue(&pdata->can_ins);
  }

  transfer.port_id        = TSMR_ODOM_GET;
  transfer.transfer_id    = tsmr_odom_transfer_id;
  transfer.payload_size   = 20;
  float odom_buff[5];
  odom_buff[0] = pdata->odom.x;
  odom_buff[1] = pdata->odom.y;
  odom_buff[2] = pdata->odom.theta;
  odom_buff[3] = pdata->odom.vx;
  odom_buff[4] = pdata->odom.wz;
  transfer.payload        = odom_buff;
  ++tsmr_odom_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
  result = canardTxPush(&pdata->can_ins, &transfer);
  canard_send_tx_queue(&pdata->can_ins);

  if(pdata->stream_power)
  {
    transfer.port_id        = TSMR_POWER_GET;
    transfer.transfer_id    = tsmr_power_transfer_id;
    transfer.payload_size   = 1;
    transfer.payload        = &pdata->about_da_power;
    ++tsmr_power_transfer_id;
    result = canardTxPush(&pdata->can_ins, &transfer);
    canard_send_tx_queue(&pdata->can_ins);
  }

  if(pdata->translation_to_set == 1)
  {
    transfer.port_id        = TSMR_T_GET;
    transfer.transfer_id    = tsmr_t_transfer_id;
    transfer.payload_size   = 4;
    transfer.payload        = &pdata->translation_to_set_value;
    ++tsmr_t_transfer_id;
    result = canardTxPush(&pdata->can_ins, &transfer);
    canard_send_tx_queue(&pdata->can_ins);
  }
  if(pdata->translation_speed_to_set == 1)
  {
    transfer.port_id        = TSMR_TS_GET;
    transfer.transfer_id    = tsmr_ts_transfer_id;
    transfer.payload_size   = 4;
    transfer.payload        = &pdata->translation_speed_to_set_value;
    ++tsmr_ts_transfer_id;
    result = canardTxPush(&pdata->can_ins, &transfer);
    canard_send_tx_queue(&pdata->can_ins);
  }
  if(pdata->rotation_to_set == 1)
  {
    transfer.port_id        = TSMR_R_GET;
    transfer.transfer_id    = tsmr_r_transfer_id;
    transfer.payload_size   = 4;
    transfer.payload        = &pdata->rotation_to_set_value;
    ++tsmr_r_transfer_id;
    result = canardTxPush(&pdata->can_ins, &transfer);
    canard_send_tx_queue(&pdata->can_ins);
  }
  if(pdata->rotation_speed_to_set == 1)
  {
    transfer.port_id        = TSMR_RS_GET;
    transfer.transfer_id    = tsmr_rs_transfer_id;
    transfer.payload_size   = 4;
    transfer.payload        = &pdata->rotation_speed_to_set_value;
    ++tsmr_rs_transfer_id;
    result = canardTxPush(&pdata->can_ins, &transfer);
    canard_send_tx_queue(&pdata->can_ins);
  }
  if(pdata->XYtheta_to_set != 0)
  {
    if(pdata->XYtheta_to_set == 1)
      transfer.port_id        = TSMR_XYT_GET;
    else
      transfer.port_id        = TSMR_XYT_BACK_GET;
    transfer.transfer_id    = tsmr_xyt_transfer_id;
    transfer.payload_size   = 12;
    transfer.payload        = &pdata->XYtheta_value_x;//works by convention
    if(pdata->XYtheta_to_set == 1)
      ++tsmr_xyt_transfer_id;
    else
      ++tsmr_xyt_back_transfer_id;
    result = canardTxPush(&pdata->can_ins, &transfer);
    canard_send_tx_queue(&pdata->can_ins);
  }


  (void)result;

  return 1;
}
