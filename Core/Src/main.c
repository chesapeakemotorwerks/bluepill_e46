/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_RxHeaderTypeDef rxHeader; //CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader; //CAN Bus Receive Header
uint8_t canRX[8] = {0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
CAN_FilterTypeDef canfil; //CAN Bus Filter
uint32_t canMailbox; //CAN Bus Mail box variable
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void delay (int a);
void setTxHeader(uint32_t CanID);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE BEGIN PD */
//Connect ADDR pin to GND and I2C slave adress will be 0X48 .
#define ADS1115_ADDRESS 0x48
unsigned char ADSwrite[6];
int16_t reading;
float voltage[2];
int voltReading[2];
const float voltageConv = 6.114 / 32768.0;
uint8_t counter = 0x00;

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
  MX_CAN_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  canfil.FilterBank = 0;
  canfil.FilterMode = CAN_FILTERMODE_IDMASK;
  canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfil.FilterIdHigh = 0;
  canfil.FilterIdLow = 0;
  canfil.FilterMaskIdHigh = 0;
  canfil.FilterMaskIdLow = 0;
  canfil.FilterScale = CAN_FILTERSCALE_32BIT;
  canfil.FilterActivation = ENABLE;
  canfil.SlaveStartFilterBank = 14;

  txHeader.DLC = 8; // Number of bites to be transmitted max- 8
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.StdId = 0x030;
  txHeader.ExtId = 0x02;
  txHeader.TransmitGlobalTime = DISABLE;

  HAL_CAN_ConfigFilter(&hcan,&canfil); //Initialize CAN Filter
  HAL_CAN_Start(&hcan); //Initialize CAN Bus
 // HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);// Initialize CAN Bus Rx Interrupt
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (counter % 10 == 0 ){
		  HAL_GPIO_WritePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin, GPIO_PIN_SET);
	  }
	  for(int i=0; i< 2; i++){
	  			ADSwrite[0] = 0x01;

	  			switch(i){
	  				case(0):
	  					ADSwrite[1] = 0xC1; //11000001
	  				break;
	  				case(1):
	  					ADSwrite[1] = 0xD1; //11010001
	  				break;
	  			//	case(2):
	  			//		ADSwrite[1] = 0xE1;
	  			//	break;
	  			//	case(3):
	  			//		ADSwrite[1] = 0xF1;
	  			//	break;
	  			}

	  			ADSwrite[2] = 0x83; //10000011 LSB

	  			HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100);
	  			ADSwrite[0] = 0x00;
	  			HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1 , ADSwrite, 1 ,100);
	  			HAL_Delay(20);

	  			HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS <<1, ADSwrite, 2, 100);
	  			reading = (ADSwrite[0] << 8 | ADSwrite[1] );
	  			if(reading < 0) {
	  				reading = 0;
	  			}
	  			//voltage[i] = reading * voltageConv;
	  			voltReading[i] = reading;



	  		}
	  uint8_t csend[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // Tx Buffer

	  /*-----------------------------------------------------------------------------*
	   * 0x316
	   *
	   * Byte 0 - 0x0D - ignition on, no errors
	   * Byte 1 - 0x00 - indexed engine torque
	   * Byte 2 - Engine Speed LSB
	   * Byte 3 - Engine Speed MSB ((HEX[MSB] * 256) + HEX[LSB]) * 0.15625
	   * Byte 4 - 0x00 - indicated engine torque
	   * Byte 5 - 0x00 - torque loss
	   * Byte 6 - 0x00 - no errors
	   * Byte 7 - 0x00 - torque MAF
	   *
	   *-----------------------------------------------------------------------------*/

	  setTxHeader(0x316);

	  csend[0] = 0x0D;
	  csend[2] = voltReading[0] & 0xFF;
	  csend[3] = (voltReading[0] >> 8 & 0xFF) * 2.77;
	  HAL_CAN_AddTxMessage(&hcan,&txHeader,csend,&canMailbox); // Send Message

	  /*-----------------------------------------------------------------------------*
	   * 0x329
	   *
	   * Byte 0 - 0x11 - CAN bus function level. Always 0x11 for MS43
	   * Byte 1 - Coolant temperature (HEX * 0.75) - 48°C. Init 0xFF
	   * Byte 2 - 0xD0 - Ambient Pressure in hPa (HEX * 2) + 598hPa. Set to 1013. Init 0x00
	   * Byte 3 - 0x0C - Engine running, no cruise
	   * Byte 4 - 0x00 - ?
	   * Byte 5 - 0x00 - TPS
	   * Byte 6 - 0x00 - no brake, brake system ok, no cruise, no shift lock
	   * Byte 7 - 0x00 - not used
	   *
	   *-----------------------------------------------------------------------------*/

	  setTxHeader(0x329);

	  csend[0] = 0x11;
	  csend[1] = (voltReading[1] >> 8 & 0xFF) * 2.77; // MSB of other axis sets temp
	  csend[2] = 0xD0;
	  csend[3] = 0x0C;
	  HAL_CAN_AddTxMessage(&hcan,&txHeader,csend,&canMailbox); // Send Message

	  /*-----------------------------------------------------------------------------*
	   * 0x545
	   *
	   * Byte 0 - 0x00 - CEL, Cruise, EML, Fuel cap lights off
	   * Byte 1 - 0x00 - fuel consumption LSB
	   * Byte 2 - 0x00 - fuel consumption MSB
	   * Byte 3 - 0x00 - Oil light, coolant overheat lights off
	   * Byte 4 - 0x48 - Oil temp HEX - 48°C. Set to 120c
	   * Byte 5 - 0x00 - not used
	   * Byte 6 - 0x00 - not used
	   * Byte 7 - 0x00 - not used
	   *
	   *-----------------------------------------------------------------------------*/

	  setTxHeader(0x545);
      csend[0] = 0x00; //reset byte to 0x00
      csend[1] = 0x00; //reset byte to 0x00
      csend[2] = 0x00; //reset byte to 0x00
      csend[3] = 0x00; //reset byte to 0x00
      csend[4] = 0x48;
      HAL_CAN_AddTxMessage(&hcan,&txHeader,csend,&canMailbox); // Send Message

      // only flash every 100ms
	  if (counter % 10 == 0 ){
		  HAL_GPIO_TogglePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin);// toggle PA3 LED
	  }
	  counter++;
	  HAL_Delay(10); // delay by 10ms
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ONBOARD_LED_Pin */
  GPIO_InitStruct.Pin = ONBOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ONBOARD_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*int _write(int file, char *ptr, int len)
{
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}*/
int __io_putchar(int ch)
{
 // Write character to ITM ch.0
 ITM_SendChar(ch);
 return(ch);
}
void delay (int a)
{
  volatile int i,j;
  for (i=0 ; i < a ; i++)
  {
     j++;
  }
  return;
}
void setTxHeader(uint32_t CanID){
	/*txHeader.DLC = 8; // Number of bites to be transmitted max- 8
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = 0x030;
	txHeader.ExtId = 0x02;
	txHeader.TransmitGlobalTime = DISABLE;*/
	txHeader.StdId = CanID;
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX); //Receive CAN bus message to canRX buffer
	printf("Received a message! ID 0x%X DLC %X MSG: 0x%X %X %X %X %X %X %X %X\n", rxHeader.StdId, rxHeader.DLC, canRX[0], canRX[1], canRX[2], canRX[3], canRX[4], canRX[5], canRX[6], canRX[7]);
	//HAL_GPIO_TogglePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin);// toggle PA3 LED

}
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

#ifdef  USE_FULL_ASSERT
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

