/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "usbd_cdc_if.h"
#include "NTC_10K_LUT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS  51200   /* 100 steps x 256 micro-steps (must match slave) */
#define MAX_BUFFER_SIZE 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t CMDSend = 0;

volatile int32_t g_Velocity = 0, g_Position = 0;
volatile bool g_VelocityState = false, g_PositionState = false;
volatile int32_t g_TempValues = 0;
volatile bool g_TempState = false;
volatile Temperature_t Slave_Temperature;

volatile bool isSent = false;
volatile bool IscmdIncomming = false;
volatile uint16_t Size_Rx = 0;

uint8_t bufferTxUART[MAX_BUFFER_SIZE] = "hello world UART\n";
uint8_t bufferRxUART[MAX_BUFFER_SIZE];
uint8_t bufferRxCmd[MAX_BUFFER_SIZE];
uint8_t bufferTxUSB[] = "hello world USB\n";

uint16_t ADC_VAL[3];
float voltage = 0.0f,vref = 0.0f, temperature = 0.0f;
float Vdda_mV = 0.0f, MCU_Temperature = 0.0f, NTC_Temperature = 0.0f;
volatile bool ADC_ConvertReady = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	isSent = true;
//	memset(bufferTxUART, 0, sizeof(bufferTxUART));
//	memcpy(bufferTxUART, bufferRxUART, sizeof(bufferRxUART));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
//  UNUSED(huart);
	if(huart->Instance == USART2)
	{

		if(Size > Size_Rx)
		{
			memcpy(bufferRxCmd, bufferRxUART, Size);
		}
		else
		{
			memcpy(bufferRxCmd, bufferRxUART, Size);
			memset(bufferRxCmd + Size, 0, Size_Rx - Size);
		}
		Size_Rx = Size;
		IscmdIncomming = true;

		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, bufferRxUART, MAX_BUFFER_SIZE);
	}
}

/* ------------------------------------------------------------------ */
/*  CAN command wrappers – one per switch-case in Slave OnCanCmd()    */
/* ------------------------------------------------------------------ */

/* 0x02 – Do Calibration */
void DoCalibration(bool _ack)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x02;
	TxData[4] = _ack ? 1u : 0u;
	CAN_Send(&TxHeader, TxData);
}

/* 0x03 – Set Current SetPoint [mA] */
void SetCurrentSetPoint(int32_t _cur)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x03;
	memcpy(TxData, &_cur, sizeof(int32_t));
	CAN_Send(&TxHeader, TxData);
}

/* 0x04 – Set Velocity SetPoint [steps/sec] */
void SetVelocitySetPoint(int32_t _vel)
{
	memcpy(TxData, &_vel, sizeof(int32_t));
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x04;
	CAN_Send(&TxHeader, TxData);
}

/* 0x05 – Set Position SetPoint */
void SetPositionSetPoint(int32_t _pos, bool _isAck)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x05;
/*
================================================================
	uint8_t* ConvertF2u8_CanBuf = (uint8_t*) &_val;
	for (uint8_t i = 0; i < 4; i++)
	{
		TxData[i] = *(ConvertF2u8_CanBuf + i);
	}
	CAN_Send(&TxHeader, TxData);
================================================================
*/
	memcpy(TxData, &_pos, sizeof(int32_t));
	TxData[4] = (uint8_t)_isAck; // Ack
	CAN_Send(&TxHeader, TxData);
}

/* 0x06 – Set Position SetPoint with Time */
void SetPositionSetPointWithTime(int32_t _pos, float _time)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x06;
	memcpy(TxData, &_pos, sizeof(int32_t));
	memcpy(TxData + 4, &_time, sizeof(float));
	CAN_Send(&TxHeader, TxData);
}

/* 0x07 – Set Position SetPoint with Velocity Limit */
void SetPositionWithVelocityLimit(int32_t _pos, int32_t _vel)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x07;
	memcpy(TxData, &_pos, sizeof(int32_t));
	memcpy(TxData + 4, &_vel, sizeof(int32_t));
	CAN_Send(&TxHeader, TxData);
}

/* 0x11 – Set Node-ID and Store to Flash */
void SetNodeId(uint32_t _nodeId, bool _store)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x11;
	memcpy(TxData, &_nodeId, sizeof(uint32_t));
	TxData[4] = _store ? 1u : 0u;
	CAN_Send(&TxHeader, TxData);
}

/* 0x12 – Set Current-Limit and Store to Flash */
void SetCurrentLimit(int32_t _limit, bool _store)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x12;
	memcpy(TxData, &_limit, sizeof(int32_t));
	TxData[4] = _store ? 1u : 0u;
	CAN_Send(&TxHeader, TxData);
}

/* 0x13 – Set Velocity-Limit and Store to Flash */
void SetVelocityLimit(int32_t _limit, bool _store)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x13;
	memcpy(TxData, &_limit, sizeof(int32_t));
	TxData[4] = _store ? 1u : 0u;
	CAN_Send(&TxHeader, TxData);
}

/* 0x14 – Set Acceleration and Store to Flash */
void SetAcceleration(int32_t _acc, bool _store)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x14;
	memcpy(TxData, &_acc, sizeof(int32_t));
	TxData[4] = _store ? 1u : 0u;
	CAN_Send(&TxHeader, TxData);
}

/* 0x15 – Apply Home-Position and Store to Flash */
void SetHomePosition(int32_t _offset, bool _store)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x15;
	memcpy(TxData, &_offset, sizeof(int32_t));
	TxData[4] = _store ? 1u : 0u;
	CAN_Send(&TxHeader, TxData);
}

/* 0x17 – Set DCE Kp and Store to Flash */
void SetDceKp(int32_t _kp, bool _store)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x17;
	memcpy(TxData, &_kp, sizeof(int32_t));
	TxData[4] = _store ? 1u : 0u;
	CAN_Send(&TxHeader, TxData);
}

/* 0x18 – Set DCE Kv and Store to Flash */
void SetDceKv(int32_t _kv, bool _store)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x18;
	memcpy(TxData, &_kv, sizeof(int32_t));
	TxData[4] = _store ? 1u : 0u;
	CAN_Send(&TxHeader, TxData);
}

/* 0x19 – Set DCE Ki and Store to Flash */
void SetDceKi(int32_t _ki, bool _store)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x19;
	memcpy(TxData, &_ki, sizeof(int32_t));
	TxData[4] = _store ? 1u : 0u;
	CAN_Send(&TxHeader, TxData);
}

/* 0x1A – Set DCE Kd and Store to Flash */
void SetDceKd(int32_t _kd, bool _store)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x1A;
	memcpy(TxData, &_kd, sizeof(int32_t));
	TxData[4] = _store ? 1u : 0u;
	CAN_Send(&TxHeader, TxData);
}

/* 0x1C – Set PID gains (kp, ki, kd) and Store to Flash */
void SetPidGains(int16_t _kp, int16_t _ki, int16_t _kd, bool _store)
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x1C;
	memcpy(TxData,     &_kp, sizeof(int16_t));
	memcpy(TxData + 2, &_ki, sizeof(int16_t));
	memcpy(TxData + 4, &_kd, sizeof(int16_t));
	TxData[6] = _store ? 1u : 0u;
	CAN_Send(&TxHeader, TxData);
}

/* 0x21 – Get Current (slave responds with current + state) */
void GetCurrent()
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x21;
	CAN_Send(&TxHeader, TxData);
}

/* 0x22 – Get Velocity (slave responds with velocity + state) */
void GetVelocity()
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x22;
	CAN_Send(&TxHeader, TxData);
}

/* 0x23 – Get Angle (slave responds with angle + state) */
void UpdateAngle()
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x23;
	CAN_Send(&TxHeader, TxData);
}

/* 0x24 – Get Offset (slave responds with encoderHomeOffset) */
void GetOffset()
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x24;
	CAN_Send(&TxHeader, TxData);
}

/* 0x25 – Get Current-Limit of slave */
void GetCurrentLimit()
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x25;
	CAN_Send(&TxHeader, TxData);
}

/* 0x26 – Get MCU & NTC temperature of slave */
void GetTemperature()
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x26;
	CAN_Send(&TxHeader, TxData);
}

/* 0x7D – Undo Configs (restore to current in-flash config & reboot) */
void UndoConfigs()
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x7D;
	CAN_Send(&TxHeader, TxData);
}

/* 0x7E – Default Configs (back to default config & reboot) */
void DefaultConfigs()
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x7E;
	CAN_Send(&TxHeader, TxData);
}

/* 0x7F – Reboot slave */
void RebootSlave()
{
	TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x7F;
	CAN_Send(&TxHeader, TxData);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_Pin)
	{
		CMDSend = CMDSend + 1;

		if(CMDSend == 2)
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			TxHeader.StdId = (TargetCAN_NodeId << 7) | 0x01;
			CAN_Send(&TxHeader, TxData);
		}
		else if(CMDSend == 3)
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			SetVelocitySetPoint(5 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
		}
		else if(CMDSend == 4)
		{
			SetVelocitySetPoint(5 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
		}
		else if(CMDSend == 5)
		{
			SetPositionSetPoint(-10 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, 0); // no ack
		}
		else if(CMDSend == 6)
		{
			SetPositionSetPoint(0 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, 1); // ack
		}
		else if(CMDSend == 7)
		{
			SetPositionWithVelocityLimit(20 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, 1 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
		}
		else if(CMDSend == 8)
		{
			DoCalibration(0);
		}
	}
}

float NTC_GetTemperature(uint16_t adc_raw)
{
    if (adc_raw == 0) adc_raw = 1;

    // B1: Tính R_NTC từ ADC (NTC phía trên, R_fixed phía dưới)
    float r_ntc = R_FIXED * (ADC_MAX / (float)adc_raw - 1.0f);

    // B2: Giới hạn trong dải bảng
    if (r_ntc >= ntc_tem_res_table[0].NTC_Resistor) return ntc_tem_res_table[0].temperature;
    if (r_ntc <= ntc_tem_res_table[NTC_TABLE_SIZE-1].NTC_Resistor) return ntc_tem_res_table[NTC_TABLE_SIZE-1].temperature;

    // B3: Tìm khoảng chứa r_ntc (bảng R giảm dần theo T tăng)
    for (uint8_t i = 0; i < NTC_TABLE_SIZE - 1; i++)
    {
        if (r_ntc <= ntc_tem_res_table[i].NTC_Resistor && r_ntc >= ntc_tem_res_table[i+1].NTC_Resistor)
        {
            // B4: Nội suy tuyến tính giữa 2 điểm gần nhất
            float r1 = ntc_tem_res_table[i].NTC_Resistor,   t1 = ntc_tem_res_table[i].temperature;
            float r2 = ntc_tem_res_table[i+1].NTC_Resistor, t2 = ntc_tem_res_table[i+1].temperature;

            return t1 + (r_ntc - r1) * (t2 - t1) / (r2 - r1);
        }
    }
    return -999.0f;
}

bool ParseCommand(char *cmd, char **_currentAction, char **_currentParam)
{
	char *saveptr = NULL;
	char *action = strtok_r(cmd,  " \r\n", &saveptr); // set
//	char *group  = strtok_r(NULL, " \r\n", &saveptr); // position
	char *param  = strtok_r(NULL, " \r\n", &saveptr); // pos
	char *value  = strtok_r(NULL, " \r\n", &saveptr); // 10
	char *extra  = strtok_r(NULL, " \r\n", &saveptr); // junk

	// 4 important token
	if(!action /*||!group*/ ||!param /*||!value*/ ||extra)
	{
		return false;
	}

	*_currentAction = action;
	*_currentParam = param;

	if((strcmp(action, "set") == 0) && (*value != 0))
	{
		// convert string to numeric values
		char *endptr = NULL;
		long val = strtol(value, &endptr, 10);

		// ignoring non numeric or redundant character
		if(endptr == value || *endptr != '\0' /*|| !isfinite(val)*/)
		{
			return false;
		}

		if(strcmp(param, "pos") == 0) // 0x05 – Set Position SetPoint
		{
			SetPositionSetPoint((int32_t)val, 0);
		}
		else if(strcmp(param, "vel") == 0)
		{
			SetVelocitySetPoint((int32_t)val);
		}
		else if(strcmp(param, "curlim") == 0)
		{
			SetCurrentLimit((int32_t)val,0);
		}
		else
		{
			return false;
		}
	}
	else if(strcmp(action, "get") == 0)
	{
		if(strcmp(param, "vel") == 0) // 0x22 – Get Velocity
		{
			GetVelocity();
		}
		else if(strcmp(param, "pos") == 0) // 0x23 – Get Angle
		{
			UpdateAngle();
		}
		else if(strcmp(param, "curlim") == 0) // 0x25 – Get Current-Limit
		{
			GetCurrentLimit();
		}
		else if(strcmp(param, "temp") == 0) // 0x25 – Get Current-Limit
		{
			GetTemperature();
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}

	return true;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance == ADC1)
	{
		ADC_ConvertReady = true;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit_DMA(&huart2, bufferTxUART, sizeof(bufferTxUART)-1);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, bufferRxUART, MAX_BUFFER_SIZE);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_VAL, 3);
  uint8_t bufferADC_UART[100];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  CDC_Transmit_FS((uint8_t*)bufferTxUSB, sizeof(bufferTxUSB));
//
//	  if ((isSent == true) && (Size_Rx != 0))
//	  {
//		  HAL_UART_Transmit_DMA(&huart2, bufferRxUART, Size_Rx);
//		  isSent = false;
//	  }

	  if(ADC_ConvertReady == true)
	  {
		  ADC_ConvertReady = false;

			vref = 3300.0f * (float)*VREFINT_CAL_ADDR / (float)ADC_VAL[0]; 	// read vrefint
			voltage = (float)ADC_VAL[1] * vref / 4095.0f; 			// read ADC at temp sensor - Vsense

			// Hai điểm hiệu chuẩn, đơn vị mV
			float V30  = (float)*TEMPSENSOR_CAL1_ADDR * 3300.0f / 4095.0f;
			float V110 = (float)*TEMPSENSOR_CAL2_ADDR * 3300.0f / 4095.0f;

			MCU_Temperature = 30.0f + (voltage - V30) * 80.0f / (V110 - V30);
			NTC_Temperature = NTC_GetTemperature(ADC_VAL[2]);

			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_VAL, 3);
	  }

//	  memset(bufferADC_UART, 0, sizeof(bufferADC_UART));
//	  snprintf((char*)bufferADC_UART, sizeof(bufferADC_UART),"Vref: %.3f MCU: %.3f NTC: %.3f ,ADC[0]:%u, ADC[1]:%u, ADC[2]:%u \n", vref, MCU_Temperature, NTC_Temperature, ADC_VAL[0], ADC_VAL[1], ADC_VAL[2]);
//	  HAL_UART_Transmit_DMA(&huart2, bufferADC_UART, strlen((char*)bufferADC_UART));

	  if(IscmdIncomming == true)
	  {
		  IscmdIncomming = false;
		  char *currentAction = NULL, *currentParam = NULL;

		  if(ParseCommand((char*)bufferRxCmd, &currentAction, &currentParam) == true)
		  {
			  if(strcmp(currentAction,"get") == 0)
			  {
//				  snprintf((char*)bufferTxUART, sizeof(bufferTxUART),"%s %s %ld Cmd OK\n", currentAction, currentParam, g_TempValues);
				  if(strcmp(currentParam,"temp") == 0)
				  {
					  snprintf((char*)bufferTxUART, sizeof(bufferTxUART),"%s %s Cmd OK MCU:%.3f NTC:%.3f\n", currentAction, currentParam, Slave_Temperature.MCU_Temperature, Slave_Temperature.NTC_Temperature);
				  }
				  else
				  {
					  snprintf((char*)bufferTxUART, sizeof(bufferTxUART),"%s %s %ld Cmd OK\n", currentAction, currentParam, g_TempValues);
				  }
			  }
			  else
			  {
				  snprintf((char*)bufferTxUART, sizeof(bufferTxUART),"%s %s Cmd OK\n", currentAction, currentParam);
			  }

			  HAL_UART_Transmit_DMA(&huart2, bufferTxUART, strlen((char*)bufferTxUART));
		  }
		  else
		  {
			  snprintf((char*)bufferTxUART, sizeof(bufferTxUART),"Cmd NOK\n");
			  HAL_UART_Transmit_DMA(&huart2, bufferTxUART, strlen((char*)bufferTxUART));
		  }
	  }

	  HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
