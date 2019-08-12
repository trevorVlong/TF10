#include "main.h"
#include "stm32f0xx_hal.h"
#include "can.h"
#include "i2c.h"
#include "adc.h"
#include "iwdg.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "main_hal.h"
#include "string.h"
#include "stdbool.h"
#include "main_hal.h"
#include <stdlib.h>
#include <string.h>

#define IN1 (1 << 0)
#define IN2 (1 << 1)
#define IN3 (1 << 2)
#define IN4 (1 << 3)
#define IN5 (1 << 4)
#define IN6 (1 << 5)
#define IN7 (1 << 6)
#define IN8 (1 << 7)
#define IN9 (1 << 8)
#define IN10 (1 << 9)
typedef union {
    uint32_t ID;
    struct {
        uint32_t SourceAddres : 8;
        uint32_t ParameterGoupNumber : 18;
        uint32_t actuatoraddres : 8;
        uint32_t Priority : 3;
    } J1939IDTypeDefine;
} J1939IDTypeDefine;

typedef union {
    uint16_t IDDIS;
    struct {
        uint16_t CanIDDIS : 16;

    } J1939IDDISTypeDefine;
} J1939IDDISTypeDefine;

typedef union {
    int8_t i8[8];
    struct {
        uint32_t Template : 8;
        uint32_t Parameter0 : 8;
        int32_t Parameter1 : 16;
        uint32_t Parameter2 : 16;
        uint32_t Parameter5 : 8;
        uint32_t Parameter6 : 8;

    } DMA20ControlMessageTypeDef;
} DMA20ControlMessageTypeDef;

typedef union {
    int8_t i8[8];
    struct {
        uint32_t measuredPosition : 14;
        uint32_t measuredCurrent : 9;
        uint32_t runningSpeed : 5;
        uint32_t voltageError : 2;
        uint32_t tempreratureError : 2;
        uint32_t motionFlag : 1;
        uint32_t backdriveFlag : 1;
        uint32_t parmeterFlag : 1;
        uint32_t saturationFlag : 1;
        uint32_t fatalErrorFlag : 1;
        uint32_t overloadFlag : 1;
    } acturatorFeedbackMessageTypeDef;
} acturatorFeedbackMessageTypeDef;
typedef union {
    int8_t i8[8];
    struct {
        uint8_t TemplateDMA1 : 8;
        uint8_t TemplateDMA2 : 8;
        uint8_t ESCandUP : 8;
        uint8_t DOWNandOK : 8;
        uint8_t Reserve1 : 8;
        uint8_t Reserve2 : 8;
        uint8_t Reserve3 : 8;

        uint32_t Reserve5 : 8;
    } DMAFeedbackMessageTypeDef;
} DMAFeedbackMessageTypeDef;

typedef union {
    int8_t i8[8];
    struct {
        uint32_t positionCommand : 14;
        uint32_t currentLimit : 9;
        uint32_t speedCommand : 5;
        uint32_t motionEnable : 1;
    } actuatorControlMessageTypeDef;
} actuatorControlMessageTypeDef;

typedef union {
    uint32_t u32[2];
    uint16_t u16[4];
    uint8_t u8[8];
    int32_t i32[2];
    int16_t i16[4];
    int8_t i8[8];
} CANMessagePackageType;

CanTxMsgTypeDef CAN_TX_Msg;
CanRxMsgTypeDef CAN_RX_Msg;
__IO uint16_t u16Timer = 50;
__IO uint16_t u16Timerstart = 1000;
__IO uint16_t u16TimerExtra = 0;
__IO uint16_t u16TimerExtra2 = 0;
uint8_t u8Data[16];
__IO uint16_t u1Timer = 0;
uint16_t u16Tx = 0;
__IO uint16_t u16Timer1 = 0;
__IO uint16_t u16Timer2 = 0;
//actuator SBrudder
volatile uint8_t backdriveFlagSBR;
volatile uint8_t runningSpeedSBR;
volatile uint16_t PositionSBR;
volatile uint16_t MoveSBR;
volatile uint16_t HuidigePositieSBR;
volatile uint32_t TackPositieSBR;
volatile uint32_t SpeedSBR;
volatile uint32_t MotionFlagSBR;
volatile uint16_t SBR;
volatile uint32_t SlowModeSBR;
volatile uint32_t SlowPositieSBR;
volatile uint32_t measuredCurrentSBR;
volatile uint32_t voltageErrorsSBR;
volatile uint32_t tempreratureErrorSBR;
volatile uint32_t overloadFlagSBR;
volatile uint32_t fatalErrorFlagSBR;
volatile uint32_t parmeterFlagSBR;
volatile uint32_t saturationFlagSBR;

//actuator BBrudder
volatile uint8_t backdriveFlagBBR;
volatile uint8_t runningSpeedBBR;
volatile uint16_t PositionBBR;
volatile uint16_t MoveBBR;
volatile uint16_t HuidigePositieBBR;
volatile uint32_t TackPositieBBR;
volatile uint32_t SpeedBBR;
volatile uint32_t MotionFlagBBR;
volatile uint16_t BBR;
volatile uint32_t SlowModeBBR;
volatile uint32_t SlowPositieBBR;
volatile uint32_t measuredCurrentBBR;
volatile uint32_t voltageErrorsBBR;
volatile uint32_t tempreratureErrorBBR;
volatile uint32_t overloadFlagBBR;
volatile uint32_t fatalErrorFlagBBR;
volatile uint32_t parmeterFlagBBR;
volatile uint32_t saturationFlagBBR;

//actuator SBmain
volatile uint8_t backdriveFlagSBM;
volatile uint8_t runningSpeedSBM;
volatile uint16_t PositionSBM;
volatile uint16_t MoveSBM;
volatile uint16_t HuidigePositieSBM;
volatile uint32_t TackPositieSBM;
volatile uint32_t SpeedSBM;
volatile uint32_t MotionFlagSBM;
volatile uint8_t SBM;
volatile uint32_t SlowModeSBM;
volatile uint32_t measuredCurrentSBM;
volatile uint32_t voltageErrorsSBM;
volatile uint32_t tempreratureErrorSBM;
volatile uint32_t overloadFlagSBM;
volatile uint32_t fatalErrorFlagSBM;
volatile uint32_t parmeterFlagSBM;
volatile uint32_t saturationFlagSBM;

//actuator BBmain
volatile uint8_t backdriveFlagBBM;
volatile uint8_t runningSpeedBBM;
volatile uint16_t PositionBBM;
volatile uint16_t MoveBBM;
volatile uint16_t HuidigePositieBBM;
volatile uint32_t TackPositieBBM;
volatile uint32_t SpeedBBM;
volatile uint32_t MotionFlagBBM;
volatile uint8_t BBM;
volatile uint32_t SlowModeBBM;
volatile uint16_t measuredCurrentBBM;
volatile uint32_t voltageErrorsBBM;
volatile uint32_t tempreratureErrorBBM;
volatile uint32_t overloadFlagBBM;
volatile uint32_t fatalErrorFlagBBM;
volatile uint32_t parmeterFlagBBM;
volatile uint32_t saturationFlagBBM;

//TACKFUNCTION
volatile uint32_t SlowModeBBR1;
volatile uint32_t SlowModeBBR2;
volatile uint32_t SlowModeSBR1;
volatile uint32_t SlowModeSBR2;
volatile uint32_t SlowModeBBM1;
volatile uint32_t SlowModeBBM2;
volatile uint32_t SlowModeSBM1;
volatile uint32_t SlowModeSBM2;
volatile uint8_t TackSpeed;
volatile uint16_t TackSpeeddis;
volatile uint32_t TackRampUp;
volatile uint32_t TackRampUpdis;
volatile uint32_t SpeedMin;
volatile uint8_t E;
volatile uint8_t Eold;
volatile uint8_t D;
volatile uint8_t Dold;
volatile uint8_t C;
volatile uint8_t Cold;
volatile uint8_t B;
volatile uint8_t Bold;
volatile uint32_t A;
volatile uint32_t Aold;
volatile acturatorFeedbackMessageTypeDef actuatorSBRStatus;
volatile acturatorFeedbackMessageTypeDef actuatorBBRStatus;
volatile acturatorFeedbackMessageTypeDef actuatorSBMStatus;
volatile acturatorFeedbackMessageTypeDef actuatorBBMStatus;
volatile DMAFeedbackMessageTypeDef DMAStatus;
volatile uint32_t DMAOK;
volatile uint32_t DMAbuttons;
volatile uint32_t tack1;
volatile uint32_t tack2;
volatile uint32_t tackpositie1;
volatile uint32_t tackpositie2;
volatile uint32_t tacksaveR;
volatile uint32_t tacksaveM;
volatile uint32_t adcValue;
volatile uint32_t IN1A;
volatile uint32_t Buttonstate;
volatile uint32_t u8WrSBR, u8RdSBR, u8WrOldSBR;
volatile uint32_t u8WrBBR, u8RdBBR, u8WrOldBBR;
volatile uint32_t u8WrSBM, u8RdSBM, u8WrOldSBM;
volatile uint32_t u8WrBBM, u8RdBBM, u8WrOldBMM;
volatile uint16_t RampUpSpeed;
volatile uint16_t RampUpSpeeddis;
volatile uint8_t temp4;
volatile uint32_t voltage;
volatile uint32_t ESCUP;
volatile uint32_t DOWNOK;
volatile uint32_t DOWNOKold;
volatile uint32_t ESCUPold;
volatile uint8_t u8Template;
volatile uint8_t OK;
volatile uint8_t ESC;
volatile uint8_t DOWN;
volatile uint8_t UP;
volatile uint8_t BBMfatalerror;
volatile uint8_t SBMfatalerror;
volatile uint8_t BBRfatalerror;
volatile uint8_t SBRfatalerror;
volatile uint8_t ERRORreset;
uint32_t mainReadButtons(void);
void SystemClock_Config(void);
void Error_Handler(void);
void CANzenderBBR(void);
void CANzenderSBR(void);
void CANzenderBBM(void);
void CANzenderSBM(void);
void CANzenderDIS(void);
void CANzenderDIStest(void);
//void CANzenderBBM (void);
//void CANzenderSBM (void);
void Buttons(void);
void savetackpositie(void);
void changetackpositie(void);
void TackButton(void);
void RampUpZero(void);
void RampUP(void);
void parameteractuatorread(void);
void fatalerror(void);
void startTemplate(void);
void ShowTemplate1(void);
void ShowTemplate2(void);
void ShowTemplate3(void);
void ShowTemplate4(void);
void ShowTemplate5(void);
void ShowTemplate6(void);
void ShowTemplate61(void);
void ShowTemplate62(void);
void ShowTemplate63(void);
void ShowTemplate64(void);
void ShowTemplate71(void);
void ShowTemplate72(void);
void ShowTemplate73(void);
void ShowTemplate74(void);
uint8_t u8Flag = 0;
uint8_t u8First;
uint8_t u8Firstwaarde;
uint8_t u8Firstknoppen;
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_CAN_Init();
    MX_IWDG_Init();
    MX_RTC_Init();
    MX_TIM16_Init();
    MX_TIM17_Init();
    MX_USART1_UART_Init();
    MX_USART3_IRDA_Init();
    MX_I2C2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_ADC_Init();

    // System init
    MainInit();
    // LED On
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Out1_HS_GPIO_Port, Out1_HS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Out2_HS_GPIO_Port, Out2_HS_Pin, GPIO_PIN_SET);
    // Set up baudrate
    hcan.Instance->BTR &= 0xFFFFFC00;
    hcan.Instance->BTR |= CAN_250K;

    adcValue = 0;
    IN1A = 0;
    /* Infinite loop */
    C = 1;
		Cold = 0;
    D = 1;
    E = 1;
    Eold = 0;
    u8First = 1;
    u8Firstwaarde = 1;
		u16Timer=500;
		u16TimerExtra2=7510;
#define WAIT1         \
    u16Timer1 = 3;    \
    while (u16Timer1) \
        ;
		#define WAIT2         \
    u16Timer2 = 100;    \
    while (u16Timer2) \
			;
    while (true) {

        if ((u16Timer == 0)&&(u16TimerExtra2==0)) { //CANVERZENDEN
            u16Timer = 50;
            //	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

            CANzenderBBR();
					WAIT1
            CANzenderSBR();
					WAIT1
            CANzenderBBM();
					WAIT1
            CANzenderSBM();
  WAIT1
            MoveSBR = 0;
            MoveBBR = 0;
            MoveSBM = 0;
            MoveBBM = 0;
        }
        voltage = (ReadAnalogInput(ADC_IN1));
        TackRampUp = 4;
        SpeedMin = 4;
        EEPROM_Read(0x0055, (uint8_t*)&TackSpeed, 1);
        EEPROM_Read(0x0065, (uint8_t*)&RampUpSpeed, 1);
        ESCUP = DMAStatus.DMAFeedbackMessageTypeDef.ESCandUP;
        DOWNOK = DMAStatus.DMAFeedbackMessageTypeDef.DOWNandOK;
				
				RampUpZero();
        parameteractuatorread();
        Buttons();
        RampUP();
        savetackpositie();
        changetackpositie();
        TackButton();
				
        if ((ESCUP & 0x1) && (D == 1)) {
            ERRORreset = 1;
            u8First = 1;
            u8Firstwaarde = 1;
        }
        else
            ERRORreset = 0;

        if ((ESCUP & 0x40) && (D == 1)) {
            ESC = 1;
            u8First = 1;
            u8Firstwaarde = 1;
        }
        else
            ESC = 0;
        if ((ESCUP & 0x80) && (D == 1)) {
            UP = 1;
            u8First = 1;
            u8Firstwaarde = 1;
        }
        else
            UP = 0;
        if ((DOWNOK & 0x01) && (D == 1)) {
            DOWN = 1;
            u8First = 1;
            u8Firstwaarde = 1;
        }
        else
            DOWN = 0;
        if ((DOWNOK & 0x02) && (D == 1)) {
            OK = 1;
            u8First = 1;
            u8Firstwaarde = 1;
        }
        else
            OK = 0;
				
				if (OK||DOWN||UP||ESC||ERRORreset)
					u16TimerExtra2=10000;
				if ((u16TimerExtra2>5000)&&(u16TimerExtra2<5010)){
					Eold=0;
					Cold=0;
					u8Template=1;
				}
        //TackSpeed
        if (UP && u8Template == 1) {
            EEPROM_Read(0x0055, (uint8_t*)&TackSpeed, 1);

            TackSpeed += 1;
            if (TackSpeed > 20)
                TackSpeed = 20;
            EEPROM_Write(0x0055, (uint8_t*)&TackSpeed, 1);
        }
        if (DOWN && u8Template == 1) {
            EEPROM_Read(0x0055, (uint8_t*)&TackSpeed, 1);

            TackSpeed -= 1;
            if (TackSpeed < 4)
                TackSpeed = 4;
            EEPROM_Write(0x0055, (uint8_t*)&TackSpeed, 1);
        }
        //RampUp
        if (DOWN && u8Template == 2) {
            EEPROM_Read(0x0065, (uint8_t*)&RampUpSpeed, 1);

            RampUpSpeed -= 10;
            if (RampUpSpeed < 50)
                RampUpSpeed = 50;
            EEPROM_Write(0x0065, (uint8_t*)&RampUpSpeed, 1);
        }

        if (UP && u8Template == 2) {
            EEPROM_Read(0x0065, (uint8_t*)&RampUpSpeed, 1);

            RampUpSpeed += 10;
            if (RampUpSpeed > 250)
                RampUpSpeed = 250;
            EEPROM_Write(0x0065, (uint8_t*)&RampUpSpeed, 1);
        }


        if (OK == 1)
            u8Template++; // uncomment and set to 0..2 for static template

        if (ESC == 1)
            u8Template--;

        //--------------------------------------------------------------------------------------------
       
        if (u8Template>=2)
					u8Template=2;
				if (u8Template<=1)
					u8Template=1;
      
        
				
           
					if (ERRORreset == 1)
            Eold = E;	
					
					if (C != Cold) {
								u8First = 1;
									Cold = C;
						}
            
						if (fatalErrorFlagBBM == 1 || fatalErrorFlagSBM == 1 || fatalErrorFlagBBR == 1 || fatalErrorFlagSBR == 1) {
						if (E != Eold) 
                fatalerror();
					}
							if (fatalErrorFlagBBM == 0 && fatalErrorFlagSBM == 0 && fatalErrorFlagBBR == 0 && fatalErrorFlagSBR == 0) {
						if (E != Eold) 
                startTemplate();
					}
						if (ERRORreset == 1)
            Eold = E;
        
      

        //--------------------------------------------------------
        if (((C == 0) || (Eold == 1)) && (u8First == 1)) {

            switch (u8Template) {
            case 0:
                ShowTemplate1();
                break;
            case 1:
                ShowTemplate1();
                break;
            case 2:
                ShowTemplate2();
                break;
            case 3:
                ShowTemplate4();
                break;
            case 4:
                ShowTemplate5();
                break;
            case 5:
                ShowTemplate5();
                break;
            case 6:
                ShowTemplate6();
                break;
            case 7:
                ShowTemplate6();
                break;
            }
        }

        RampUpZero();
        parameteractuatorread();
        Buttons();
        RampUP();
        savetackpositie();
        changetackpositie();
        TackButton();
				if (u8First==1)
						u16Timer=1000;
				u8First = 0;
    }

// Watchdog refresh, see file main.h:70
#if (PRODUCTION_VERSION == 1)
    HAL_IWDG_Refresh(&hiwdg);
#warning Production version, Debugging not possible! <<<<<<<<<<<<<<<<<<<<<
#else
#warning Debug version without watch dog! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#endif

    /* USER CODE END 3 */
}

void parameteractuatorread(void)
{

    HuidigePositieSBR = actuatorSBRStatus.acturatorFeedbackMessageTypeDef.measuredPosition;
    HuidigePositieBBR = actuatorBBRStatus.acturatorFeedbackMessageTypeDef.measuredPosition;
    HuidigePositieSBM = actuatorSBMStatus.acturatorFeedbackMessageTypeDef.measuredPosition;
    HuidigePositieBBM = actuatorBBMStatus.acturatorFeedbackMessageTypeDef.measuredPosition;

    measuredCurrentSBR = actuatorSBRStatus.acturatorFeedbackMessageTypeDef.measuredCurrent;
    measuredCurrentBBR = actuatorBBRStatus.acturatorFeedbackMessageTypeDef.measuredCurrent;
    measuredCurrentSBM = actuatorSBMStatus.acturatorFeedbackMessageTypeDef.measuredCurrent;
    measuredCurrentBBM = actuatorBBMStatus.acturatorFeedbackMessageTypeDef.measuredCurrent;

    runningSpeedSBR = actuatorSBRStatus.acturatorFeedbackMessageTypeDef.runningSpeed;
    runningSpeedBBR = actuatorBBRStatus.acturatorFeedbackMessageTypeDef.runningSpeed;
    runningSpeedSBM = actuatorSBMStatus.acturatorFeedbackMessageTypeDef.runningSpeed;
    runningSpeedBBM = actuatorBBMStatus.acturatorFeedbackMessageTypeDef.runningSpeed;

    voltageErrorsSBR = actuatorSBRStatus.acturatorFeedbackMessageTypeDef.voltageError;
    voltageErrorsBBR = actuatorBBRStatus.acturatorFeedbackMessageTypeDef.voltageError;
    voltageErrorsSBM = actuatorSBMStatus.acturatorFeedbackMessageTypeDef.voltageError;
    voltageErrorsBBM = actuatorBBMStatus.acturatorFeedbackMessageTypeDef.voltageError;

    tempreratureErrorSBR = actuatorSBRStatus.acturatorFeedbackMessageTypeDef.tempreratureError;
    tempreratureErrorBBR = actuatorBBRStatus.acturatorFeedbackMessageTypeDef.tempreratureError;
    tempreratureErrorSBM = actuatorSBMStatus.acturatorFeedbackMessageTypeDef.tempreratureError;
    tempreratureErrorBBM = actuatorBBMStatus.acturatorFeedbackMessageTypeDef.tempreratureError;

    MotionFlagSBR = actuatorSBRStatus.acturatorFeedbackMessageTypeDef.motionFlag;
    MotionFlagBBR = actuatorBBRStatus.acturatorFeedbackMessageTypeDef.motionFlag;
    MotionFlagSBM = actuatorSBMStatus.acturatorFeedbackMessageTypeDef.motionFlag;
    MotionFlagBBM = actuatorBBMStatus.acturatorFeedbackMessageTypeDef.motionFlag;

    overloadFlagSBR = actuatorBBMStatus.acturatorFeedbackMessageTypeDef.overloadFlag;
    overloadFlagBBR = actuatorBBMStatus.acturatorFeedbackMessageTypeDef.overloadFlag;
    overloadFlagSBM = actuatorBBMStatus.acturatorFeedbackMessageTypeDef.overloadFlag;
    overloadFlagBBM = actuatorBBMStatus.acturatorFeedbackMessageTypeDef.overloadFlag;

    backdriveFlagSBR = actuatorSBRStatus.acturatorFeedbackMessageTypeDef.backdriveFlag;
    backdriveFlagBBR = actuatorBBRStatus.acturatorFeedbackMessageTypeDef.backdriveFlag;
    backdriveFlagSBM = actuatorSBMStatus.acturatorFeedbackMessageTypeDef.backdriveFlag;
    backdriveFlagBBM = actuatorBBMStatus.acturatorFeedbackMessageTypeDef.backdriveFlag;

    fatalErrorFlagSBR=actuatorSBRStatus.acturatorFeedbackMessageTypeDef.fatalErrorFlag;
    fatalErrorFlagBBR=actuatorBBRStatus.acturatorFeedbackMessageTypeDef.fatalErrorFlag;
    fatalErrorFlagSBM=actuatorSBMStatus.acturatorFeedbackMessageTypeDef.fatalErrorFlag;
    fatalErrorFlagBBM=actuatorBBMStatus.acturatorFeedbackMessageTypeDef.fatalErrorFlag;

    parmeterFlagSBR = actuatorSBRStatus.acturatorFeedbackMessageTypeDef.parmeterFlag;
    parmeterFlagBBR = actuatorBBRStatus.acturatorFeedbackMessageTypeDef.parmeterFlag;
    parmeterFlagSBM = actuatorSBMStatus.acturatorFeedbackMessageTypeDef.parmeterFlag;
    parmeterFlagBBM = actuatorBBMStatus.acturatorFeedbackMessageTypeDef.parmeterFlag;

    saturationFlagSBR = actuatorSBRStatus.acturatorFeedbackMessageTypeDef.saturationFlag;
    saturationFlagBBR = actuatorBBRStatus.acturatorFeedbackMessageTypeDef.saturationFlag;
    saturationFlagSBM = actuatorSBMStatus.acturatorFeedbackMessageTypeDef.saturationFlag;
    saturationFlagBBM = actuatorBBMStatus.acturatorFeedbackMessageTypeDef.saturationFlag;

    Buttonstate = mainReadButtons();
}

void RampUP(void)
{
    if (u16TimerExtra == 0) {
        u16TimerExtra = RampUpSpeed;

        if (SBR == 1) {
            SlowModeSBR++;

            SpeedSBR = SlowModeSBR;

            if (SpeedMin > SlowModeSBR) {
                SpeedSBR = SpeedMin;
            }

            if (SlowModeSBR > 20) {
                SpeedSBR = 20;
            }
        }
        if (BBR == 1) {
            SlowModeBBR++;

            SpeedBBR = SlowModeBBR;

            if (SpeedMin > SlowModeBBR) {
                SpeedBBR = SpeedMin;
            }

            if (SlowModeBBR > 20) {
                SpeedBBR = 20;
            }
        }
        if (SBM == 1) {
            SlowModeSBM++;

            SpeedSBM = SlowModeSBM;

            if (SpeedMin > SlowModeSBM) {
                SpeedSBM = SpeedMin;
            }

            if (SlowModeSBM > 20) {
                SpeedSBM = 20;
            }
        }
        if (BBM == 1) {
            SlowModeBBM++;

            SpeedBBM = SlowModeBBM;

            if (SpeedMin > SlowModeBBM) {
                SpeedBBM = SpeedMin;
            }

            if (SlowModeBBM > 20) {
                SpeedBBM = 20;
            }
        }
    }
}

void RampUpZero(void)
{

    if (SBR == 0) {
        SlowModeSBR = 0;
        SpeedSBR = SpeedMin;
    }
    if (BBR == 0) {
        SlowModeBBR = 0;
        SpeedBBR = SpeedMin;
    }
    if (SBM == 0) {
        SlowModeSBM = 0;
        SpeedSBM = SpeedMin;
    }
    if (BBM == 0) {
        SlowModeBBM = 0;
        SpeedBBM = SpeedMin;
    }
}

void TackButton(void)
{
    if (A == 1) {

        PositionBBR = TackPositieBBR;
        PositionSBR = TackPositieSBR;
        PositionBBM = TackPositieBBM;
        PositionSBM = TackPositieSBM;
        MoveSBR = 1;
        MoveBBR = 1;
        MoveSBM = 1;
        MoveBBM = 1;

        if (PositionBBR > TackPositieSBR) {
            SlowModeBBR1 = HuidigePositieBBR - TackPositieSBR;
            SlowModeBBR2 = TackPositieBBR - HuidigePositieBBR;
        }

        if (PositionBBR < TackPositieSBR) {
            SlowModeBBR1 = TackPositieSBR - HuidigePositieBBR;
            SlowModeBBR2 = HuidigePositieBBR - TackPositieBBR;
        }
        if (SlowModeBBR1 < SlowModeBBR2)
            SlowModeBBR = SlowModeBBR1;
        else
            SlowModeBBR = SlowModeBBR2;

        SpeedBBR = SlowModeBBR / TackRampUp;
        if (SpeedBBR > TackSpeed) {
            SpeedBBR = TackSpeed;
        }
        if (SpeedBBR < SpeedMin) {
            SpeedBBR = SpeedMin;
        }

        if (PositionSBR > TackPositieBBR) {
            SlowModeSBR1 = HuidigePositieSBR - TackPositieBBR;
            SlowModeSBR2 = TackPositieSBR - HuidigePositieSBR;
        }

        if (PositionSBR < TackPositieBBR) {
            SlowModeSBR1 = TackPositieBBR - HuidigePositieSBR;
            SlowModeSBR2 = HuidigePositieSBR - TackPositieSBR;
        }
        if (SlowModeSBR1 < SlowModeSBR2)
            SlowModeSBR = SlowModeSBR1;
        else
            SlowModeSBR = SlowModeSBR2;
				
        SpeedSBR = SlowModeSBR / TackRampUp; // set SBR ram speed from distance and delay
        // thresholding max/min
				if (SpeedSBR > TackSpeed) {
            SpeedSBR = TackSpeed;
        }
        if (SpeedSBR < SpeedMin) {
            SpeedSBR = SpeedMin;
        }

        if (PositionBBM > TackPositieSBM) {
            SlowModeBBM1 = HuidigePositieBBM - TackPositieSBM;
            SlowModeBBM2 = TackPositieBBM - HuidigePositieBBM;
        }

        if (PositionBBM < TackPositieSBM) {
            SlowModeBBM1 = TackPositieSBM - HuidigePositieBBM;
            SlowModeBBM2 = HuidigePositieBBM - TackPositieBBM;
        }
        if (SlowModeBBM1 < SlowModeBBM2)
            SlowModeBBM = SlowModeBBM1;
        else
            SlowModeBBM = SlowModeBBM2;

        SpeedBBM = SlowModeBBM / TackRampUp;
        if (SpeedBBM > TackSpeed) {
            SpeedBBM = TackSpeed;
        }
        if (SpeedBBM < SpeedMin) {
            SpeedBBM = SpeedMin;
        }

        if (PositionSBM > TackPositieBBM) {
            SlowModeSBM1 = HuidigePositieSBM - TackPositieBBM;
            SlowModeSBM2 = TackPositieSBM - HuidigePositieSBM;
        }

        if (PositionSBM < TackPositieBBM) {
            SlowModeSBM1 = TackPositieBBM - HuidigePositieSBM;
            SlowModeSBM2 = HuidigePositieSBM - TackPositieSBM;
        }
        if (SlowModeSBM1 < SlowModeSBM2)
            SlowModeSBM = SlowModeSBM1;
        else
            SlowModeSBM = SlowModeSBM2;

        SpeedSBM = SlowModeSBM / TackRampUp;
        if (SpeedSBM > TackSpeed) {
            SpeedSBM = TackSpeed;
        }
        if (SpeedSBM < SpeedMin) {
            SpeedSBM = SpeedMin;
        }

        if ((Buttonstate != IN10) && (MotionFlagSBR == 0) && (MotionFlagBBR == 0) && (MotionFlagSBM == 0) && (MotionFlagSBM == 0)) {

            A = 0;
        }
    }
}

void changetackpositie(void)
{

    if (Buttonstate != IN10) //||(D==0))
    {
        B = 0;
    }
    if ((Buttonstate == IN10) && (B == 0)) //||(D==1))&&(B==0))
    {

        EEPROM_Read(0x0005, (uint8_t*)&u8RdSBM, 4);

        TackPositieSBM = u8RdSBM;

        EEPROM_Read(0x0010, (uint8_t*)&u8RdBBM, 4);

        TackPositieBBM = u8RdBBM;

        EEPROM_Read(0x0015, (uint8_t*)&u8RdSBR, 4);

        TackPositieSBR = u8RdSBR;

        EEPROM_Read(0x0020, (uint8_t*)&u8RdBBR, 4);

        TackPositieBBR = u8RdBBR;

        tacksaveR = TackPositieSBR;
        TackPositieSBR = TackPositieBBR;
        TackPositieBBR = tacksaveR;

        tacksaveM = TackPositieSBM;
        TackPositieSBM = TackPositieBBM;
        TackPositieBBM = tacksaveM;

        u8WrSBM = TackPositieSBM;
        EEPROM_Write(0x0005, (uint8_t*)&u8WrSBM, 4);

        u8WrBBM = TackPositieBBM;
        EEPROM_Write(0x0010, (uint8_t*)&u8WrBBM, 4);

        u8WrSBR = TackPositieSBR;
        EEPROM_Write(0x0015, (uint8_t*)&u8WrSBR, 4);

        u8WrBBR = TackPositieBBR;
        EEPROM_Write(0x0020, (uint8_t*)&u8WrBBR, 4);

        B = 1;
        A = 1;
    }
}

void savetackpositie(void)
{
    //{uint32_t u32WrSBR, u32RdSBR;

    if ((SBR == 1) && (MotionFlagSBR == 0)) { //READTACKPOSITION SBR
        TackPositieSBR = HuidigePositieSBR;
        u8WrSBR = TackPositieSBR;
        EEPROM_Write(0x0015, (uint8_t*)&u8WrSBR, 4);
        SBR = 0;
    }
    if ((BBR == 1) && (MotionFlagBBR == 0)) { //READTACKPOSITION BBR
        TackPositieBBR = HuidigePositieBBR;
        BBR = 0;
        u8WrBBR = TackPositieBBR;
        EEPROM_Write(0x0020, (uint8_t*)&u8WrBBR, 4);
    }
    if ((SBM == 1) && (MotionFlagSBM == 0)) { //READTACKPOSITION SBM
        TackPositieSBM = HuidigePositieSBM;
        SBM = 0;
        u8WrSBM = TackPositieSBM;
        EEPROM_Write(0x0005, (uint8_t*)&u8WrSBM, 4);
    }
    if ((BBM == 1) && (MotionFlagBBM == 0)) { //READTACKPOSITION BBM
        TackPositieBBM = HuidigePositieBBM;
        BBM = 0;
        u8WrBBM = TackPositieBBM;
        EEPROM_Write(0x0010, (uint8_t*)&u8WrBBM, 4);
    }
}
void Buttons(void)
{

    if (Buttonstate & IN5) { //SBR+
        PositionSBR = 1000;
        MoveSBR = 1;
        SBR = 1;
        A = 0;
      
    }
    else
        fatalErrorFlagSBR = 0;

    if (Buttonstate & IN4) { //SBR-
        PositionSBR = 0;
        MoveSBR = 1;
        SBR = 1;
        A = 0;
       
    }

    else
        fatalErrorFlagBBR = 0;
    if (Buttonstate & IN3) { //BBR+
        PositionBBR = 1000;
        MoveBBR = 1;
        BBR = 1;
        A = 0;
        
    }
    else
        fatalErrorFlagSBM = 0;
    if (Buttonstate & IN2) { //BBR-
        PositionBBR = 0;
        MoveBBR = 1;
        BBR = 1;
        A = 0;
       
    }
    else
        fatalErrorFlagBBM = 0;
    if (Buttonstate & IN9) { //SBM+
        PositionSBM = 1000;
        MoveSBM = 1;
        SBM = 1;
        A = 0;
    }

    if (Buttonstate & IN8) { //SBM-
        PositionSBM = 0;
        MoveSBM = 1;
        SBM = 1;
        A = 0;
    }

    if (Buttonstate & IN7) { //BBM+
        PositionBBM = 1000;
        MoveBBM = 1;
        BBM = 1;
        A = 0;
    }
    if (Buttonstate & IN6) { //BBM-
        PositionBBM = 0;
        MoveBBM = 1;
        BBM = 1;
        A = 0;
    }
}

uint32_t mainReadButtons(void)
{
    //BUTTONREAD
    uint32_t Buttonstate = 0;

    Buttonstate |= (ReadAnalogInput(ADC_IN1) > 5000) ? (0) : 0;
    Buttonstate |= (ReadAnalogInput(ADC_IN2) > 5000) ? (1 << 1) : 0;
    Buttonstate |= (ReadAnalogInput(ADC_IN3) > 5000) ? (1 << 2) : 0;
    Buttonstate |= (ReadAnalogInput(ADC_IN4) > 5000) ? (1 << 3) : 0;
    Buttonstate |= (ReadAnalogInput(ADC_IN5) > 5000) ? (1 << 4) : 0;
    Buttonstate |= (ReadAnalogInput(ADC_IN6) > 5000) ? (1 << 5) : 0;
    Buttonstate |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) << 6);
    Buttonstate |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) << 7);
    Buttonstate |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) << 8);
    Buttonstate |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) << 9);

    return Buttonstate;
}

actuatorControlMessageTypeDef sendControlSBR(void)
{
    actuatorControlMessageTypeDef FeedbackCan;

    FeedbackCan.actuatorControlMessageTypeDef.positionCommand = PositionSBR;
    FeedbackCan.actuatorControlMessageTypeDef.currentLimit = 200;
    FeedbackCan.actuatorControlMessageTypeDef.speedCommand = SpeedSBR;
    FeedbackCan.actuatorControlMessageTypeDef.motionEnable = MoveSBR;

    return FeedbackCan;
}
actuatorControlMessageTypeDef sendControlBBR(void)
{
    actuatorControlMessageTypeDef FeedbackCan;

    FeedbackCan.actuatorControlMessageTypeDef.positionCommand = PositionBBR;
    FeedbackCan.actuatorControlMessageTypeDef.currentLimit = 200;
    FeedbackCan.actuatorControlMessageTypeDef.speedCommand = SpeedBBR;
    FeedbackCan.actuatorControlMessageTypeDef.motionEnable = MoveBBR;

    return FeedbackCan;
}

actuatorControlMessageTypeDef sendControlSBM(void)
{
    actuatorControlMessageTypeDef FeedbackCan;

    FeedbackCan.actuatorControlMessageTypeDef.positionCommand = PositionSBM;
    FeedbackCan.actuatorControlMessageTypeDef.currentLimit = 200;
    FeedbackCan.actuatorControlMessageTypeDef.speedCommand = SpeedSBM;
    FeedbackCan.actuatorControlMessageTypeDef.motionEnable = MoveSBM;

    return FeedbackCan;
}

actuatorControlMessageTypeDef sendControlBBM(void)
{
    actuatorControlMessageTypeDef FeedbackCan;

    FeedbackCan.actuatorControlMessageTypeDef.positionCommand = PositionBBM;
    FeedbackCan.actuatorControlMessageTypeDef.currentLimit = 200;
    FeedbackCan.actuatorControlMessageTypeDef.speedCommand = SpeedBBM;
    FeedbackCan.actuatorControlMessageTypeDef.motionEnable = MoveBBM;

    return FeedbackCan;
}
void CANzenderSBR(void)
{
    static J1939IDTypeDefine J1939ID;
    static CANMessagePackageType CANTXMessage;
    uint8_t u8I;

    actuatorControlMessageTypeDef messageHolder = sendControlSBR();

    memcpy(&CANTXMessage, &messageHolder, 8);

    // Fill CAN-ID
    J1939ID.J1939IDTypeDefine.Priority = 4;

    J1939ID.J1939IDTypeDefine.ParameterGoupNumber = 0x00EF00 + 0x15; //ontvanger id

    J1939ID.J1939IDTypeDefine.SourceAddres = 0xff;

    // Fill header
    hcan.pTxMsg->IDE = CAN_ID_EXT;
    hcan.pTxMsg->ExtId = J1939ID.ID;
    hcan.pTxMsg->DLC = 8; // datalengte
    // Transfer data
    for (u8I = 0; u8I < 8; u8I++)
        hcan.pTxMsg->Data[u8I] = CANTXMessage.u8[u8I];
    // Send it by CAN

    HAL_CAN_Transmit(&hcan, 1000);
}

void CANzenderBBR(void)
{
    static J1939IDTypeDefine J1939ID;
    static CANMessagePackageType CANTXMessage;
    uint8_t u8I;

    actuatorControlMessageTypeDef messageHolder = sendControlBBR();

    memcpy(&CANTXMessage, &messageHolder, 8);

    // Fill CAN-ID
    J1939ID.J1939IDTypeDefine.Priority = 4;

    J1939ID.J1939IDTypeDefine.ParameterGoupNumber = 0x00EF00 + 0x17; //ontvanger id

    J1939ID.J1939IDTypeDefine.SourceAddres = 0xff;

    // Fill header
    hcan.pTxMsg->IDE = CAN_ID_EXT;
    hcan.pTxMsg->ExtId = J1939ID.ID;
    hcan.pTxMsg->DLC = 8; // datalengte
    // Transfer data
    for (u8I = 0; u8I < 8; u8I++)
        hcan.pTxMsg->Data[u8I] = CANTXMessage.u8[u8I];
    // Send it by CAN

    HAL_CAN_Transmit(&hcan, 1000);
}

void CANzenderSBM(void)
{
    static J1939IDTypeDefine J1939ID;
    static CANMessagePackageType CANTXMessage;
    uint8_t u8I;

    actuatorControlMessageTypeDef messageHolder = sendControlSBM();

    memcpy(&CANTXMessage, &messageHolder, 8);

    // Fill CAN-ID
    J1939ID.J1939IDTypeDefine.Priority = 4;

    J1939ID.J1939IDTypeDefine.ParameterGoupNumber = 0x00EF00 + 0x14; //ontvanger id

    J1939ID.J1939IDTypeDefine.SourceAddres = 0xff;

    // Fill header
    hcan.pTxMsg->IDE = CAN_ID_EXT;
    hcan.pTxMsg->ExtId = J1939ID.ID;
    hcan.pTxMsg->DLC = 8; // datalengte
    // Transfer data
    for (u8I = 0; u8I < 8; u8I++)
        hcan.pTxMsg->Data[u8I] = CANTXMessage.u8[u8I];
    // Send it by CAN

    HAL_CAN_Transmit(&hcan, 1000);
}

void CANzenderBBM(void)
{
    static J1939IDTypeDefine J1939ID;
    static CANMessagePackageType CANTXMessage;
    uint8_t u8I;

    actuatorControlMessageTypeDef messageHolder = sendControlBBM();

    memcpy(&CANTXMessage, &messageHolder, 8);

    // Fill CAN-ID
    J1939ID.J1939IDTypeDefine.Priority = 4;

    J1939ID.J1939IDTypeDefine.ParameterGoupNumber = 0x00EF00 + 0x16; //ontvanger id

    J1939ID.J1939IDTypeDefine.SourceAddres = 0xff;

    // Fill header
    hcan.pTxMsg->IDE = CAN_ID_EXT;
    hcan.pTxMsg->ExtId = J1939ID.ID;
    hcan.pTxMsg->DLC = 8; // datalengte
    // Transfer data
    for (u8I = 0; u8I < 8; u8I++)
        hcan.pTxMsg->Data[u8I] = CANTXMessage.u8[u8I];
    // Send it by CAN

    HAL_CAN_Transmit(&hcan, 1000);
}

void fatalerror(void)
{
    hcan.pTxMsg->IDE = CAN_ID_STD;
    hcan.pTxMsg->StdId = 0x7FB;
    hcan.pTxMsg->DLC = 8;
    if (u8First) {
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 0; // Load Template
        (*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 4; // Template 2 (1V_BAR)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0x00; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2
        if (fatalErrorFlagBBR == 1) {
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 1; // Variable 0 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Starb", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 2; // Variable 1 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "oard ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 3; // Variable 0 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Rudde", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 4; // Variable 1 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "r    ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2
        }
        if (fatalErrorFlagSBR == 1) {
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 1; // Variable 0 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Port ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 2; // Variable 1 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Rudde", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 3; // Variable 0 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "r    ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 4; // Variable 1 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "     ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2
        }
        if (fatalErrorFlagSBM == 1) {

            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 1; // Variable 0 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Starb", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 2; // Variable 1 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "oard ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 3; // Variable 0 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Main ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 4; // Variable 1 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "     ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2
        }
        if (fatalErrorFlagBBM == 1) {
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 1; // Variable 0 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Port ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 2; // Variable 1 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Main ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 3; // Variable 0 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "    ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 4; // Variable 1 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "     ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2
        }

        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 5; // Variable 0 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Actua", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 6; // Variable 1 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "tor F", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 7; // Variable 0 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "ATAL ", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 8; // Variable 1 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "ERROR", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 9; // Variable 0 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "see M", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 10; // Variable 1 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "anual", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 11; // Variable 0 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "!", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 12; // Variable 1 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "   ", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 13; // Button 0 (Esc)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 14; // Button 1 (Up)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0x00; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2(*(DMA20_t*)(hcan.pTxMsg->Data))
            .u16Multiplexer
            = 15; // Button 2 (Down)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0x00; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 16; // Button 3 (OK)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0x00; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        D = 0 ;
    }
}
void startTemplate(void)
{
    hcan.pTxMsg->IDE = CAN_ID_STD;
    hcan.pTxMsg->StdId = 0x7FB;
    hcan.pTxMsg->DLC = 8;
    if (u8First) {
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 0; // Load Template
        (*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 4; // Template 2 (1V_BAR)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0x00; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2
     
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 1; // Variable 0 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "DNA  ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 2; // Variable 1 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "    ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 3; // Variable 0 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "     ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2

                (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                    .u16Multiplexer
                = 4; // Variable 1 (Text)
            strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "     ", 5);
            (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
            HAL_CAN_Transmit(&hcan, 10);
            WAIT2
        
       

        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 5; // Variable 0 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "TF10 ", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 6; // Variable 1 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Trima", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 7; // Variable 0 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "ran", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 8; // Variable 1 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "     ", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 9; // Variable 0 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Foil ", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 10; // Variable 1 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "Contr", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 11; // Variable 0 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "ol Sy", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 12; // Variable 1 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "stem ", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 13; // Button 0 (Esc)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 14; // Button 1 (Up)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0x00; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2(*(DMA20_t*)(hcan.pTxMsg->Data))
            .u16Multiplexer
            = 15; // Button 2 (Down)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0x00; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT2

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 16; // Button 3 (OK)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0x00; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        D = 0;
    }
}
void ShowTemplate1(void)
{
    //u16Tx = u16Tx + 10 + (rand() & 0xFFF);
    //if (u16Tx > 30000)
    //	u16Tx = 1;
    // Send it by CAN
    hcan.pTxMsg->IDE = CAN_ID_STD;
    hcan.pTxMsg->StdId = 0x7FB;
    hcan.pTxMsg->DLC = 8;
    if (u8First) {
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 0; // Load Template
        (*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 2; // Template 2 (1V_BAR)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 1; // Variable 0 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "TACKS", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 2; // Variable 1 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "PEED ", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 3; // Variable 2 (Parameter)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 1; //
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1
    }

    (*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 4; // Variable 3 (Value)
    (*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = TackSpeeddis = TackSpeed * 500; //
    (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
    HAL_CAN_Transmit(&hcan, 10);
    WAIT1

    if (u8First) {
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 5; // Variable 4 (Unit)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 1; // %
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1
    }

    (*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 6; // Variable 5 (Bargraph)
    (*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 50; // %
    (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
    HAL_CAN_Transmit(&hcan, 10);
    WAIT1

    if (u8First) {
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 7; // Button 0 (Esc)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 8; // Button 1 (Up)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 9; // Button 2 (Down)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 10; // Button 3 (OK)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        D = 0;
    }
}

void ShowTemplate2(void)
{
    //u16Tx = u16Tx + 10 + (rand() & 0xFFF);
    //if (u16Tx > 30000)
    //	u16Tx = 1;
    // Send it by CAN
    hcan.pTxMsg->IDE = CAN_ID_STD;
    hcan.pTxMsg->StdId = 0x7FB;
    hcan.pTxMsg->DLC = 8;
    if (u8First) {
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 0; // Load Template
        (*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 2; // Template 2 (1V_BAR)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 1; // Variable 0 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "RAMP ", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1

            (*(DMA20Char_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 2; // Variable 1 (Text)
        strncpy((*(DMA20Char_t*)(hcan.pTxMsg->Data)).ch5, "UP   ", 5);
        (*(DMA20Char_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF;
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 3; // Variable 2 (Parameter)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 2; //
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1
    }

    (*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 4; // Variable 3 (Value)
    (*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = RampUpSpeeddis = RampUpSpeed * 50 - 2500; //
    (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
    HAL_CAN_Transmit(&hcan, 10);
    WAIT1

    if (u8First) {
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u16Multiplexer = 5; // Variable 4 (Unit)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 1; // %
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 6; // Variable 5 (Bargraph)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).i32Value = 100; // %
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 7; // Button 0 (Esc)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 8; // Button 1 (Up)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 9; // Button 2 (Down)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        WAIT1

            (*(DMA20_t*)(hcan.pTxMsg->Data))
                .u16Multiplexer
            = 10; // Button 3 (OK)
        (*(DMA20_t*)(hcan.pTxMsg->Data)).u8Visible = 0xFF; // Visible
        HAL_CAN_Transmit(&hcan, 10);
        D = 0;
    }
}
void ShowTemplate3(void)
{
}

void ShowTemplate4(void)
{
    
}

void ShowTemplate5(void)
{
    
}

void ShowTemplate6(void)
{
    
}

void ShowTemplate7(void)
{
   
}





void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
    // Implement CAN RX
    if (hcan->pRxMsg->IDE == CAN_ID_EXT) {
        switch (hcan->pRxMsg->ExtId) {
        case 0x19EFFF14:
            memcpy((void*)&actuatorSBMStatus, hcan->pRxMsg->Data, 8);
            break;

        case 0x19EFFF15:
            memcpy((void*)&actuatorSBRStatus, hcan->pRxMsg->Data, 8);
            break;

        case 0x19EFFF16:
            memcpy((void*)&actuatorBBMStatus, hcan->pRxMsg->Data, 8);
            break;

        case 0x19EFFF17:
            memcpy((void*)&actuatorBBRStatus, hcan->pRxMsg->Data, 8);
            break;
        }
    }
    if (hcan->pRxMsg->IDE == CAN_ID_STD) {
        D = 1;
        switch (hcan->pRxMsg->StdId) {
        case 0x7FC:
            memcpy((void*)&DMAStatus, hcan->pRxMsg->Data, 8);
            break;
        default:
            break;
        }
    }

    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
}

void HAL_SYSTICK_Callback(void)
{

    // First timer
    if (u16Timer > 0)
        u16Timer--;

    // Second timer
    // First timer
    if (u16TimerExtra > 0)
        u16TimerExtra--;

    if (u16Timerstart > 0)
        u16Timerstart--;

    if (u16TimerExtra2 > 0)
        u16TimerExtra2--;

    if (u16Timer1 > 0)
        u16Timer1--;
		 if (u16Timer2 > 0)
        u16Timer2--;
}

void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14 | RCC_OSCILLATORTYPE_LSI
        | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
    RCC_OscInitStruct.HSI14CalibrationValue = 16;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_USART1
        | RCC_PERIPHCLK_RTC;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV32;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    /**Configure the Systick interrupt time 
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick 
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while (1) {
    };
}
