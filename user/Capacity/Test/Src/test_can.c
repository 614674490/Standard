
                                     /*以下代码是can的滤波器*/
#include "test_can.h"
#include "can.h"
void CanFilter_Init(CAN_HandleTypeDef* hcan)
{
  CAN_FilterConfTypeDef canfilter;
  static CanTxMsgTypeDef  Tx1Message;
  static CanRxMsgTypeDef  Rx1Message;
//  static CanTxMsgTypeDef  Tx2Message;
//  static CanRxMsgTypeDef  Rx2Message;
  
  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
  
  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000;
  
  canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;// CAN_FilterFIFO0;
  canfilter.FilterActivation = ENABLE;
  canfilter.BankNumber = 14;
	/*can1和CAN2使用不同的滤波器*/
//  if(hcan == &hcan1)
//  {
    canfilter.FilterNumber = 0;
    hcan->pTxMsg = &Tx1Message;
    hcan->pRxMsg = &Rx1Message;
//  }
//  if(hcan == &hcan2)
//  {
//    canfilter.FilterNumber = 14;
//    hcan->pTxMsg = &Tx2Message;
//    hcan->pRxMsg = &Rx2Message;
//  }
	
  
  HAL_CAN_ConfigFilter(hcan, &canfilter);
  
}

