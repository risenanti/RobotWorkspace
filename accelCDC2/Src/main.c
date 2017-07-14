#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "setup.hpp"

#include "usbd_cdc_if.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
TIM_OC_InitTypeDef htimPwmPulse;

#define REG_X_AXIS_L 0x28
#define REG_X_AXIS_H 0x29
#define REG_Y_AXIS_L 0x2A
#define REG_Y_AXIS_H 0x2B
// control reg1 (20h) - ODR: 25Hz, X,Y,Z Axis enabled.
uint8_t ConfigReg1[2] = { 0x20, 0x37 }; /*0011 0000  0011 0111*/
uint8_t accelDataX[2];
uint8_t accelDataY[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

static void setupExternInterPins(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void pwmChannelOneStart(void);
void pwmChannelTwoStart(void);
void pwmChannelThreeStart(void);
void pwmChannelFourStart(void);

void pwmChannelOneStop(void);
void pwmChannelTwoStop(void);
void pwmChannelThreeStop(void);
void pwmChannelFourStop(void);

void pwmChannelOneSet(uint16_t set);
void pwmChannelTwoSet(uint16_t set);
void pwmChannelThreeSet(uint16_t set);
void pwmChannelFourSet(uint16_t set);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t buff[10];
uint8_t str1[2];
uint8_t retStrcmp;
uint32_t motor1Encoder;

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  htimPwmPulse.OCMode = TIM_OCMODE_PWM1;
  htimPwmPulse.Pulse = 500;
  htimPwmPulse.OCPolarity = TIM_OCPOLARITY_HIGH;
  htimPwmPulse.OCFastMode = TIM_OCFAST_DISABLE;

  MX_GPIO_Init();
  setupExternInterPins();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_SPI5_Init();
  MX_TIM3_Init();


  //while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)0x32, (uint8_t*)&ConfigReg1[0], 2, 10000) != HAL_OK) { }
  HAL_Delay(1);

  while (1)
  {
	  uint8_t testDataToSend[13] = {'H', 'E', 'L','L','O',' ','W','O','R','L','D', '\r'};

	  //HAL_I2C_Mem_Read(&hi2c1, 0x32, REG_X_AXIS_L, I2C_MEMADD_SIZE_8BIT, &accelDataX[0], 1, 10000);
	  //HAL_I2C_Mem_Read(&hi2c1, 0x32, REG_X_AXIS_H, I2C_MEMADD_SIZE_8BIT, &accelDataX[1], 1, 10000);

	  //HAL_I2C_Mem_Read(&hi2c1, 0x32, REG_Y_AXIS_L, I2C_MEMADD_SIZE_8BIT, &accelDataY[0], 1, 10000);
	  //HAL_I2C_Mem_Read(&hi2c1, 0x32, REG_Y_AXIS_H, I2C_MEMADD_SIZE_8BIT, &accelDataY[1], 1, 10000);


	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7));
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6));
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3));
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2));

	  //CDC_Transmit_FS(testDataToSend, 12);

	  //uint8_t buff[6];
	  CDC_Receive_FS_User(buff, 10); //Receives from usb port every update

	  strncpy (str1, buff, 3 );

	  retStrcmp = strcmp(str1,"M1O");
	  if(retStrcmp==0)
	  {
		  pwmChannelOneSet(500);
		  pwmChannelOneStart();
	  }

	  retStrcmp = strcmp(str1,"M1F");
	  if(retStrcmp==0)
	  {
		  pwmChannelOneStop();
	  }

	  retStrcmp = strcmp(str1,"M2O");
	  if(retStrcmp==0)
	  {
		  pwmChannelTwoSet(500);
		  pwmChannelTwoStart();
	  }

	  retStrcmp = strcmp(str1,"M2F");
	  if(retStrcmp==0)
	  {
		  pwmChannelTwoStop();
	  }

	  retStrcmp = strcmp(str1,"M3O");
	  if(retStrcmp==0)
	  {
		  pwmChannelThreeSet(500);
		  pwmChannelThreeStart();
	  }

	  retStrcmp = strcmp(str1,"M3F");
	  if(retStrcmp==0)
	  {
		  pwmChannelThreeStop();
	  }

	  retStrcmp = strcmp(str1,"M4O");
	  if(retStrcmp==0)
	  {
		  pwmChannelFourSet(500);
		  pwmChannelFourStart();
	  }

	  retStrcmp = strcmp(str1,"M4F");
	  if(retStrcmp==0)
	  {
		  pwmChannelFourStop();
	  }
  }
}


static void setupExternInterPins(void)
{
	/*Setup PD7 & 6 as interupt pin*/
	  GPIO_InitTypeDef   GPIO_InitStructure;

	  __HAL_RCC_GPIOD_CLK_ENABLE();

	  /* Configure PA0 pin as input floating */
	  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStructure.Pull = GPIO_NOPULL;
	  GPIO_InitStructure.Pin = GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_3|GPIO_PIN_2;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);


	  //GPIO_InitStructure.Pin = GPIO_PIN_6;
	  //HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
	  //HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
	  //HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}


/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
     PB12   ------> I2S2_WS
     PC7   ------> I2S3_MCK
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
*/


/* USER CODE BEGIN 4 */
void pwmChannelOneStart(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}
void pwmChannelTwoStart(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}
void pwmChannelThreeStart(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}
void pwmChannelFourStart(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void pwmChannelOneStop(void)
{
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}
void pwmChannelTwoStop(void)
{
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}
void pwmChannelThreeStop(void)
{
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
}
void pwmChannelFourStop(void)
{
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}

void pwmChannelOneSet(uint16_t set)
{
	htimPwmPulse.Pulse = set;
	HAL_TIM_PWM_ConfigChannel(&htim3, &htimPwmPulse, TIM_CHANNEL_1);
}
void pwmChannelTwoSet(uint16_t set)
{
	htimPwmPulse.Pulse = set;
	HAL_TIM_PWM_ConfigChannel(&htim3, &htimPwmPulse, TIM_CHANNEL_2);
}
void pwmChannelThreeSet(uint16_t set)
{
	htimPwmPulse.Pulse = set;
	HAL_TIM_PWM_ConfigChannel(&htim3, &htimPwmPulse, TIM_CHANNEL_3);
}
void pwmChannelFourSet(uint16_t set)
{
	htimPwmPulse.Pulse = set;
	HAL_TIM_PWM_ConfigChannel(&htim3, &htimPwmPulse, TIM_CHANNEL_4);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}


void _Error_Handler(char * file, int line)
{
  while(1) {}
}

#ifdef USE_FULL_ASSERT


void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}
#endif
