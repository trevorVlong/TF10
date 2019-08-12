#ifndef __MAIN_HAL_H
#define __MAIN_HAL_H

#include "main.h"
#include "stm32f0xx_hal.h"

extern CanTxMsgTypeDef CAN_TX_Msg;
extern CanRxMsgTypeDef CAN_RX_Msg;
extern CanTxMsgTypeDef CAN_TX_Msg;
extern CanRxMsgTypeDef CAN_RX_Msg;
extern ADC_HandleTypeDef hadc;
extern CAN_HandleTypeDef hcan;
extern I2C_HandleTypeDef hi2c2;
extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart1;



HAL_StatusTypeDef EEPROM_Read(uint16_t BaseAddress, uint8_t* Data, uint16_t Size);
HAL_StatusTypeDef EEPROM_Write(uint16_t BaseAddress, uint8_t* Data, uint16_t Size);
uint16_t ReadAnalogInput (uint8_t Channel);
void DIn7ResetCounter ( void );
uint16_t DIn7ReadCounter ( void );
uint16_t DIn8ReadFrequency ( void );
uint16_t DIn8ReadDutyCycle ( void );
void MainInit ( void );

#endif /* __MAIN_HAL_H */
