
#include "main.h"
#include "stm32f0xx_hal.h"
#include "main_hal.h"

/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  Read from EEPROM 
  * @param  BaseAddress: EEPROM start address
	*         Data: Target memory for read
	*					Size: Bytes to read
  * @retval Status
  */
HAL_StatusTypeDef EEPROM_Read(uint16_t BaseAddress, uint8_t* Data, uint16_t Size)
{
    HAL_StatusTypeDef Result = HAL_OK;
    uint16_t u16ByteCounter = 0;
	
    while (u16ByteCounter < Size && Result == HAL_OK)
    {
        uint16_t u16BytesToRead = Size - u16ByteCounter;

        if (u16BytesToRead < EEPROM_BUFFER_SIZE)
        {
            // one Page or less
            Result = HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDRESS, BaseAddress + u16ByteCounter, I2C_MEMADD_SIZE_16BIT, &Data[u16ByteCounter], u16BytesToRead, EEPROM_TIMEOUT);
            u16ByteCounter += u16BytesToRead;
        }
        else
        {
            // more Pages
            Result = HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDRESS, BaseAddress + u16ByteCounter, I2C_MEMADD_SIZE_16BIT, &Data[u16ByteCounter], EEPROM_BUFFER_SIZE, EEPROM_TIMEOUT);
            u16ByteCounter += EEPROM_BUFFER_SIZE;
        }
    }
    return Result;
}

/**
  * @brief  Write to EEPROM 
  * @param  BaseAddress: EEPROM start address
	*         Data: data source for write
	*					Size: Bytes to write
  * @retval Status
  */
HAL_StatusTypeDef EEPROM_Write(uint16_t BaseAddress, uint8_t* Data, uint16_t Size)
{
    uint16_t u16ByteCounter = 0;
    HAL_StatusTypeDef Result = HAL_OK;
    while (u16ByteCounter < Size && Result == HAL_OK)
    {
        uint16_t u16BytesToWrite = Size - u16ByteCounter;

        if (u16BytesToWrite < EEPROM_BUFFER_SIZE)
        {
            // one Page or less
            Result = HAL_I2C_Mem_Write(&hi2c2, EEPROM_ADDRESS, BaseAddress + u16ByteCounter, I2C_MEMADD_SIZE_16BIT, &Data[u16ByteCounter], u16BytesToWrite, EEPROM_TIMEOUT);
            u16ByteCounter += u16BytesToWrite;
        }
        else
        {
            // more Pages
            Result = HAL_I2C_Mem_Write(&hi2c2, EEPROM_ADDRESS, BaseAddress + u16ByteCounter, I2C_MEMADD_SIZE_16BIT, &Data[u16ByteCounter], EEPROM_BUFFER_SIZE, EEPROM_TIMEOUT);
            u16ByteCounter += EEPROM_BUFFER_SIZE;
        }
        HAL_Delay(EEPROM_WRITE_TIME);
    }
    return Result;
}

/**
  * @brief  Read Voltage from Analog input (~80 µs)
  * @param  Channel [1..3]
  * @retval Voltage [mV]
  */
uint16_t ReadAnalogInput ( uint8_t Channel)
{
	uint16_t u16DR;
	
	hadc.Instance->CHSELR = ADC_CHSELR_CHANNEL(Channel-1);
	HAL_ADC_Start(&hadc);
	if ( HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK )
	{	
		u16DR = HAL_ADC_GetValue(&hadc);
		HAL_ADC_Stop(&hadc);
		return (uint16_t) ( (uint32_t)30800 * (uint32_t)u16DR / 4096 );
	}
	else
		return 0xFFFF; // Error
}

/**
  * @brief  Reset Counter DIN4
  * @param  none
  * @retval none
  */
void DIn7ResetCounter ( void )
{
	htim1.Instance->CNT = 0;
}

/**
  * @brief  Read Counter DIN4
  * @param  none
  * @retval none
  */
uint16_t DIn7ReadCounter ( void )
{
	return htim1.Instance->CNT;
}

/**
  * @brief  Read Frequency from DIN5
  * @param  none
  * @retval Frequency [Hz]
  */
uint16_t DIn8ReadFrequency ( void )
{ 
	return (16000000 / (htim2.Instance->PSC+1)) / htim2.Instance->CCR1;
}

/**
  * @brief  Read Duty cycle from DIN5
  * @param  none
  * @retval Duty Cycle [%]
  */
uint16_t DIn8ReadDutyCycle ( void )
{ 
	return (uint8_t)((uint32_t)htim2.Instance->CCR2 * (uint32_t)100 / (uint32_t)htim2.Instance->CCR1); 
}


void MainInit ( void )
{
	CAN_FilterConfTypeDef sFilterConfig;

  /* MCU Configuration----------------------------------------------------------*/

//  __HAL_IRDA_DISABLE(&hirda3);
//	hirda3.Instance->CR3 |= USART_CR3_OVRDIS;
//  __HAL_IRDA_ENABLE(&hirda3);


	// Instance CAN RX and TX
	// Replace CAN_250K in MX_CAN_Init to change the baudrate
	hcan.pTxMsg = &CAN_TX_Msg;
	hcan.pRxMsg = &CAN_RX_Msg;
	// create open filter
	sFilterConfig.FilterIdHigh = 0;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 0;
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	// Interrupt config
	HAL_CAN_Receive_IT(&hcan, 0);
	// Change CAN from Silent to Active mode
	HAL_GPIO_WritePin(CAN_S_GPIO_Port, CAN_S_Pin, GPIO_PIN_RESET);
	
	// Calibrate and Start ADC
	HAL_ADCEx_Calibration_Start(&hadc);
	
	// LS-PWM Sample 500 µs On + 500 µs Off
	// 16 MHz / (PSC+1) = 1 MHz
	// 1 MHz / (ARR+1) = 1 kHz => f = 1 ms
	// DC = 50 % => CCR1 = ARR * 50 / 100
	htim17.Instance->PSC = 16-1;
	htim17.Instance->ARR = 1000-1;
	htim17.Instance->CCR1 = (1000-1) * 50 / 100;
	HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1);
	
	// DIN6/DIN7
	HAL_TIM_Base_MspInit(&htim1);
	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);
	htim1.Instance->ARR = 0xFFFF;
	DIn7ResetCounter();
	
	// DIN7 Frequency counter
	HAL_TIM_Base_MspInit(&htim2);
	htim2.Instance->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1;
	htim2.Instance->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_TS_0	| TIM_SMCR_SMS_2;
	htim2.Instance->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;
	htim2.Instance->DIER |= TIM_DIER_CC1IE;
	htim2.Instance->CR1 |= TIM_CR1_CEN;	
	htim2.Instance->ARR = 0xFFFF;
	
	// PSC = Prescaler (/1../65536)
	// CNT = Counter (0..65565)
	// 16 MHz -> /PSC  -> Gate -> CNT 
	// Input >--------------^
	// Sample: measure maximal frequency of 100 Hz with 1 % accuracy 
	// Requirement: 100 Hz with 1 % accuracy = 10000 Tics/s -> CNT = 10000/s -> fIN_CNT >= 10 kHz
	// Solution: 16 MHz / 10 kHz = 1600 -> PSC=1600-1
	// minimal frequency: CNTmax/10 kHz for overflow: 65535/10000 = 6,55 s = 0,153 Hz
	
	// 20 kHz 5%
	// 20*20=400 kHz
	// 16 MHz/400 kHz = 40 --> PSC = 40-1
	// minimal frequency: CNTmax/10 kHz for overflow: 65535/400000 = 0,164 s = 6,1 Hz
	htim2.Instance->PSC = 40-1;
	// Result calculation:
	// f = 10 kHz / htim2.Instance->CCR1
	// DC = htim2.Instance->CCR1 * 100 / htim2.Instance->CCR1 [%]

	// f = 400 kHz / htim2.Instance->CCR1
}
// ===========================================================================





