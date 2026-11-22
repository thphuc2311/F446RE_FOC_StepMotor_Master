/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
    STATE_STOP,
    STATE_FINISH,
    STATE_RUNNING,
    STATE_OVERLOAD,
    STATE_STALL,
    STATE_NO_CALIB
} State_t;

typedef struct
{
	float MCU_Temperature;
	float NTC_Temperature;
}Temperature_t;
extern volatile Temperature_t Slave_Temperature;

extern volatile int32_t g_Velocity, g_Position;
extern volatile bool g_VelocityState, g_PositionState;
extern volatile int32_t g_TempValues;
extern volatile bool g_TempState;
extern volatile bool Cmd_Reponse;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* 0x02 – Do Calibration */
extern void DoCalibration(bool _ack);

/* 0x03 – Set Current SetPoint [mA] */
extern void SetCurrentSetPoint(int32_t _cur);

/* 0x04 – Set Velocity SetPoint [steps/sec] */
void SetVelocitySetPoint(int32_t _vel);

/* 0x05 – Set Position SetPoint */
extern void SetPositionSetPoint(int32_t _pos, bool _isAck);

/* 0x06 – Set Position SetPoint with Time */
extern void SetPositionSetPointWithTime(int32_t _pos, float _time);

/* 0x07 – Set Position SetPoint with Velocity Limit */
extern void SetPositionWithVelocityLimit(int32_t _pos, int32_t _vel);

/* 0x11 – Set Node-ID and Store to Flash */
extern void SetNodeId(uint32_t _nodeId, bool _store);

/* 0x12 – Set Current-Limit and Store to Flash */
extern void SetCurrentLimit(int32_t _limit, bool _store);

/* 0x13 – Set Velocity-Limit and Store to Flash */
extern void SetVelocityLimit(int32_t _limit, bool _store);

/* 0x14 – Set Acceleration and Store to Flash */
extern void SetAcceleration(int32_t _acc, bool _store);

/* 0x15 – Apply Home-Position and Store to Flash */
extern void SetHomePosition(int32_t _offset, bool _store);

/* 0x17 – Set DCE Kp and Store to Flash */
extern void SetDceKp(int32_t _kp, bool _store);

/* 0x18 – Set DCE Kv and Store to Flash */
extern void SetDceKv(int32_t _kv, bool _store);

/* 0x19 – Set DCE Ki and Store to Flash */
extern void SetDceKi(int32_t _ki, bool _store);

/* 0x1A – Set DCE Kd and Store to Flash */
extern void SetDceKd(int32_t _kd, bool _store);

/* 0x1C – Set PID gains (kp, ki, kd) and Store to Flash */
extern void SetPidGains(int16_t _kp, int16_t _ki, int16_t _kd, bool _store);

/* 0x21 – Get Current (slave responds with current + state) */
extern void GetCurrent();

/* 0x22 – Get Velocity (slave responds with velocity + state) */
extern void GetVelocity();

/* 0x23 – Get Angle (slave responds with angle + state) */
extern void UpdateAngle();

/* 0x24 – Get Offset (slave responds with encoderHomeOffset) */
extern void GetOffset();

/* 0x25 – Get Current-Limit of slave */
extern void GetCurrentLimit();

/* 0x26 – Get MCU & NTC temperature of slave */
extern void GetTemperature();

/* 0x7D – Undo Configs (restore to current in-flash config & reboot) */
extern void UndoConfigs();

/* 0x7E – Default Configs (back to default config & reboot) */
extern void DefaultConfigs();

/* 0x7F – Reboot slave */
extern void RebootSlave();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
