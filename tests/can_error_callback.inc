void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if ((hcan->ErrorCode & (HAL_CAN_ERROR_TX_TERR0 | HAL_CAN_ERROR_TX_TERR1 | HAL_CAN_ERROR_TX_TERR2 |
                          HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_ALST2)) != 0u)
  {
    s_canTxErrors++;
    CAN_Send_Error(hcan->ErrorCode);
  }

  hcan->ErrorCode = HAL_CAN_ERROR_NONE;   /* HAL 内部只做 |=，不清零，这里手动清 */
}