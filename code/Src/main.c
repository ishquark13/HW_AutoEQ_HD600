/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);

float l_a0, l_a1, l_a2, l_b1, l_b2, lin_z1, lin_z2, lout_z1, lout_z2;
float r_a0, r_a1, r_a2, r_b1, r_b2, rin_z1, rin_z2, rout_z1, rout_z2;
float a_00, a_01, a_02, b_01, b_02;
float a_10, a_11, a_12, b_11, b_12;
float a_20, a_21, a_22, b_21, b_22;
float a_30, a_31, a_32, b_31, b_32;
float a_40, a_41, a_42, b_41, b_42;
float a_50, a_51, a_52, b_51, b_52;
float a_60, a_61, a_62, b_61, b_62;
float a_70, a_71, a_72, b_71, b_72;
float a_80, a_81, a_82, b_81, b_82;
float a_90, a_91, a_92, b_91, b_92;

uint16_t rxBuf[8];
uint16_t txBuf[8];
int main(void)
{
 HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();

  HAL_I2SEx_TransmitReceive_DMA (&hi2s2, txBuf, rxBuf, 4);


  //left-channel, High-Pass, 1kHz, fs=96kHz, q=0.7
  l_a0 = 0.9543457485325094f;
  l_a1 = -1.9086914970650188f;
  l_a2 = 0.9543457485325094f;
  l_b1 = -1.9066459797557103f;
  l_b2 = 0.9107370143743273f;

  //right-channel, Low-Pass, 1kHz, fs)96 kHz, q=0.7
  r_a0 = 0.0010227586546542474f;
  r_a1 = 0.002045517309308495f;
  r_a2 = 0.0010227586546542474f;
  r_b1 = -1.9066459797557103f;
  r_b2 = 0.9107370143743273f;

  // HD600 Filter 1: ON LSC Fc 105 Hz Gain 6.4 dB Q 0.70
  a_00 = 1.002166885885848;
  a_01 = -1.9902556715873885;
  a_02 = 0.988186980588655;
  b_01 = -1.9902812695170031;
  b_02 = 0.9903282685448882;

  // HD600 Filter 2: ON PK Fc 9090 Hz Gain 4.3 dB Q 2.25
  a_10 = 1.0709469081822147;
  a_11 = -1.4729186894638182;
  a_12 = 0.7075481251435357;
  b_11 = -1.4729186894638182;
  b_12 = 0.7784950333257503;

  // HD600 Filter 3: ON PK Fc 141 Hz Gain -2.6 dB Q 0.80
  a_20 = 0.9980028304703323;
  a_21 = -1.984474840194307;
  a_22 = 0.9865565155222485;
  b_21 = -1.984474840194307;
  b_22 = 0.9845593459925809;

  // HD600 Filter 4: ON PK Fc 514 Hz Gain 0.9 dB Q 1.17
  a_30 = 1.00154702875083;
  a_31 = -1.970544014273223;
  a_32 = 0.9701125752134796;
  b_31 = -1.970544014273223;
  b_32 = 0.9716596039643095;

  // HD600 Filter 5: ON PK Fc 3018 Hz Gain -1.8 dB Q 2.45
  a_40 = 0.9912107719290015;
  a_41 = -1.8690184542025916;
  a_42 = 0.9148719089560465;
  b_41 = -1.8690184542025916;
  b_42 = 0.906082680885048;

  // HD600 Filter 6: ON HSC Fc 10000 Hz Gain -1.5 dB Q 0.70
  a_50 = 0.8731333757133074;
  a_51 = -0.9685046801373233;
  a_52 = 0.34764005643962353;
  b_51 = -1.1748465804602355;
  b_52 = 0.42711533247584305;

  // HD600 Filter 7: ON PK Fc 5584 Hz Gain -2.7 dB Q 4.65
  a_60 = 0.9866875200318461;
  a_61 = -1.7748385254998431;
  a_62 = 0.9136590190830277;
  b_61 = -1.7748385254998431;
  b_62 = 0.9003465391148741;

  // HD600 Filter 8: ON PK Fc 1402 Hz Gain -0.8 dB Q 2.52
  a_70 = 0.9982802189680681;
  a_71 = -1.9526595925529386;
  a_72 = 0.9626290335047876;
  b_71 = -1.9526595925529386;
  b_72 = 0.9609092524728559;

  // HD600 Filter 9: ON PK Fc 4419 Hz Gain 2.2 dB Q 6.00
  a_80 = 1.006691862827416;
  a_81 =-1.8724290527416163;
  a_82 = 0.9468770999820032;
  b_81 = -1.8724290527416163;
  b_82 = 0.9535689628094193;

  // HD600 Filter 10: ON PK Fc 7643 Hz Gain 1.6 dB Q 5.27
  a_90 = 1.008803573425046;
  a_91 = -1.6785575418757952;
  a_92 = 0.9041462890488169;
  b_91 = -1.6785575418757952;
  b_92 = 0.9129498624738628;

  while (1)
  {

  }

}

int Calc_IIR(int inSample, float a0, float a1, float a2, float b1, float b2) {
	float inSampleF = (float)inSample;
	float outSampleF =
			a0 * inSampleF
			+a1 * lin_z1
			+a2 * lin_z2
			-b1 * lout_z1
			-b2 * lout_z2;
	lin_z2 = lin_z1;
	lin_z1 = inSampleF;
	lout_z2 = lout_z1;
	lout_z1 = outSampleF;

	return (int) outSampleF;
}

int Calc_IIR_Left (int inSample) {
	float inSampleF = (float)inSample;
	float outSampleF =
			l_a0 * inSampleF
			+ l_a1 * lin_z1
			+ l_a2 * lin_z2
			- l_b1 * lout_z1
			- l_b2 * lout_z2;
	lin_z2 = lin_z1;
	lin_z1 = inSampleF;
	lout_z2 = lout_z1;
	lout_z1 = outSampleF;

	return (int) outSampleF;
}

int Calc_IIR_Right (int inSample) {
	float inSampleF = (float)inSample;
	float outSampleF =
			r_a0 * inSampleF
			+ r_a1 * rin_z1
			+ r_a2 * rin_z2
			- r_b1 * rout_z1
			- r_b2 * rout_z2;
	rin_z2 = rin_z1;
	rin_z1 = inSampleF;
	rout_z2 = rout_z1;
	rout_z1 = outSampleF;

	return (int) outSampleF;
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s){

	//restore signed 24 bit sample from 16-bit buffers
	int lSample = (int) (rxBuf[0]<<16)|rxBuf[1];
	int rSample = (int) (rxBuf[2]<<16)|rxBuf[3];

	// divide by 2 (rightshift) -> -6dB per sample
	lSample = lSample>>1;
	rSample = rSample>>1;

	//sum to mono
	lSample = rSample + lSample;
	rSample = lSample;

	//run HP on left channel and LP on right channel
	//lSample = Calc_IIR_Left(lSample);
    //rSample = Calc_IIR_Right(rSample);

	// recursive calls??

	lSample = Calc_IIR(lSample, a_00, a_01, a_02, b_01,b_02);
	lSample = Calc_IIR(lSample, a_10, a_11, a_12, b_11,b_12);
	lSample = Calc_IIR(lSample, a_20, a_21, a_22, b_21,b_22);
	lSample = Calc_IIR(lSample, a_30, a_31, a_32, b_31,b_32);
	lSample = Calc_IIR(lSample, a_40, a_41, a_42, b_41,b_42);
	lSample = Calc_IIR(lSample, a_50, a_51, a_52, b_51,b_52);
	lSample = Calc_IIR(lSample, a_60, a_61, a_62, b_61,b_62);
	lSample = Calc_IIR(lSample, a_70, a_71, a_72, b_71,b_72);
	lSample = Calc_IIR(lSample, a_80, a_81, a_82, b_81,b_82);
	lSample = Calc_IIR(lSample, a_90, a_91, a_92, b_91,b_92);

	rSample = lSample;


	//restore to buffer
	txBuf[0] = (lSample>>16)&0xFFFF;
	txBuf[1] = lSample&0xFFFF;
	txBuf[2] = (rSample>>16)&0xFFFF;
	txBuf[3] = rSample&0xFFFF;
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){

	//restore signed 24 bit sample from 16-bit buffers
	int lSample = (int) (rxBuf[4]<<16)|rxBuf[5];
	int rSample = (int) (rxBuf[6]<<16)|rxBuf[7];

	// divide by 2 (rightshift) -> -6sdB per sample
	lSample = lSample>>1;
	rSample = rSample>>1;

	//sum to mono
	lSample = rSample + lSample;
	rSample = lSample;

	//run HP on left channel and LP on right channel
	//lSample = Calc_IIR_Left(lSample);
	//rSample = Calc_IIR_Right(rSample);

	// recursive calls??

	lSample = Calc_IIR(lSample, a_00, a_01, a_02, b_01,b_02);
	lSample = Calc_IIR(lSample, a_10, a_11, a_12, b_11,b_12);
	lSample = Calc_IIR(lSample, a_20, a_21, a_22, b_21,b_22);
	lSample = Calc_IIR(lSample, a_30, a_31, a_32, b_31,b_32);
	lSample = Calc_IIR(lSample, a_40, a_41, a_42, b_41,b_42);
	lSample = Calc_IIR(lSample, a_50, a_51, a_52, b_51,b_52);
	lSample = Calc_IIR(lSample, a_60, a_61, a_62, b_61,b_62);
	lSample = Calc_IIR(lSample, a_70, a_71, a_72, b_71,b_72);
	lSample = Calc_IIR(lSample, a_80, a_81, a_82, b_81,b_82);
	lSample = Calc_IIR(lSample, a_90, a_91, a_92, b_91,b_92);

	rSample = lSample;
	//restore to buffer
	txBuf[4] = (lSample>>16)&0xFFFF;
	txBuf[5] = lSample&0xFFFF;
	txBuf[6] = (rSample>>16)&0xFFFF;
	txBuf[7] = rSample&0xFFFF;
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 
                           PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC10 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
