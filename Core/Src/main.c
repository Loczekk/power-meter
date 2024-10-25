/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_DIFFERENCE 0.02f  // 5% tolerancji
#define ERROR_THRESHOLD 10    // Liczba błędów, po której wartości są akceptowane
#define ILOSC_PROBEK 7      // ilosc probek do usredniania
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t adcValue_FWR;
uint32_t adcValue_REF;
volatile float attenuation_FWR = 0.0f;  // Dodatkowy czynnik tłumienia padajacej mocy
int comparator = 0;
uint32_t value_FWR[1];  // Bufor na jedną próbkę (DMA nadpisuje value_ADC1[0])
float samples_FWR[ILOSC_PROBEK];   // Bufor kołowy na 3 ostatnie próbki
int sample_index_FWR = 0;  // Indeks śledzący miejsce zapisu nowej próbki
int error_count_FWR = 0;  // Licznik błędów dla odrzuconych próbek
float previous_sample_FWR = 0.0f;  // Przechowujemy poprzednią próbkę do porównania
float average_voltage_FWD = 0.0f;
float power_FWD = 0.0f;
float powerWatts_FWD = 0.0f;

volatile float attenuation_REF = 0.0f;  // Dodatkowy czynnik tłumienia odbitej mocy
uint32_t value_REF[1];  // Bufor na jedną próbkę (DMA nadpisuje value_ADC1[0])
float samples_REF[ILOSC_PROBEK];   // Bufor kołowy na 3 ostatnie próbki
int sample_index_REF= 0;  // Indeks śledzący miejsce zapisu nowej próbki
int error_count_REF = 0;  // Licznik błędów dla odrzuconych próbek
float previous_sample_REF = 0.0f;  // Przechowujemy poprzednią próbkę do porównania
float average_voltage_REF = 0.0f;
float power_REF = 0.0f;
float powerWatts_REF = 0.0f;

void collect_sample_FWR();
//void collect_sample_REF();
int is_valid_sample(float sample1, float sample2);
float calculate_average_FWR();
float calculate_average_REF();
float calculateWFS(float power_incident_watts, float power_reflected_watts);
float WFS = 0.0f;
//void process_adc_values();
float dBm_to_Watts(float dBm);
char myString_FWD[30];
char myString2_FWD[30];
char myString_REF[30];
char myString2_REF[30];
char myString_WFS[30];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	char myText[] = "POWER METER V2.0";

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
  MX_I2C3_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  //IIR_Init(&filt, alpha);
  ssd1306_SetCursor(5, 5);
  ssd1306_WriteString(myText, Font_7x10, White);
  ssd1306_UpdateScreen();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_TIM_Base_Start(&htim4);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)value_FWR, 1);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)value_REF, 1);
  HAL_Delay (500);
  HAL_GPIO_WritePin(OHM51_GPIO_Port, OHM51_Pin, 1);
  HAL_GPIO_WritePin(OHM500_GPIO_Port, OHM500_Pin, 0);

  while (1)
  {
	  while (comparator != 1) {
		  __WFI();  // Czekanie na przerwanie
	  }

	  collect_sample_FWR();  // Zbieraj próbkę za każdym przerwaniem EXTI
	  comparator = 2;
     	    ssd1306_Fill(Black);
     	    ssd1306_SetCursor(5, 5);
     	    ssd1306_WriteString(myString_FWD, Font_7x10, White);
     	    ssd1306_SetCursor(5, 25);
     	    ssd1306_WriteString(myString2_FWD, Font_7x10, White);
     	    ssd1306_SetCursor(60, 5);
     	    ssd1306_WriteString(myString_REF, Font_7x10, White);
     	    ssd1306_SetCursor(60, 25);
     	    ssd1306_WriteString(myString2_REF, Font_7x10, White);
     	    ssd1306_SetCursor(60, 45);
     	    ssd1306_WriteString(myString_WFS, Font_7x10, White);
     	    ssd1306_UpdateScreen();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OHM500_Pin|OHM51_Pin|TEST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Comp_input_Pin */
  GPIO_InitStruct.Pin = Comp_input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Comp_input_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OHM500_Pin OHM51_Pin */
  GPIO_InitStruct.Pin = OHM500_Pin|OHM51_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TEST_Pin */
  GPIO_InitStruct.Pin = TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Funkcja zbierająca próbkę w reakcji na przerwanie
void collect_sample_FWR() {
    float new_sample_FWR = 3.3f * adcValue_FWR / 4096.0f;  // Przelicz na napięcie
    float new_sample_REF = 3.3f * adcValue_REF / 4096.0f;  // Przelicz na napięcie

    // Sprawdzanie nowej próbki w porównaniu do poprzedniej
    if (is_valid_sample(previous_sample_FWR, new_sample_FWR)) {
        // Jeśli próbka jest poprawna, resetujemy licznik błędów
        error_count_FWR = 0;

        // Zapisujemy nową próbkę w buforze kołowym (nadpisujemy najstarszą)
        samples_FWR[sample_index_FWR] = new_sample_FWR;
        sample_index_FWR = (sample_index_FWR + 1) % ILOSC_PROBEK;  // Przesuwamy wskaźnik cyklicznie

        // Aktualizujemy poprzednią próbkę
        previous_sample_FWR = new_sample_FWR;

        // Możemy teraz przetworzyć średnią z próbek
        average_voltage_FWD = calculate_average_FWR();

        power_FWD = average_voltage_FWD / -0.0123f + 21.910569f;
        power_FWD += attenuation_FWR;
        powerWatts_FWD = dBm_to_Watts(power_FWD);

        sprintf(myString_FWD, "%.3f", average_voltage_FWD);
        sprintf(myString2_FWD, "%.3f", power_FWD);

// 	    ssd1306_Fill(Black);
// 	    ssd1306_SetCursor(5, 5);
// 	    ssd1306_WriteString(myString_FWD, Font_7x10, White);
// 	    ssd1306_SetCursor(5, 25);
// 	    ssd1306_WriteString(myString2_FWD, Font_7x10, White);
// 	    ssd1306_UpdateScreen();

    } else {
        // W przypadku błędnej próbki
        error_count_FWR++;

        if (error_count_FWR >= ERROR_THRESHOLD) {
            // Akceptujemy wartości mimo błędów, jeśli licznik błędów przekroczył próg
            samples_FWR[sample_index_FWR] = new_sample_FWR;
            sample_index_FWR = (sample_index_FWR + 1) % ILOSC_PROBEK;

            // Aktualizujemy poprzednią próbkę
            previous_sample_FWR = new_sample_FWR;

            average_voltage_FWD = calculate_average_FWR();

            power_FWD = average_voltage_FWD / -0.0123f + 21.910569f;
            power_FWD += attenuation_FWR;
            powerWatts_FWD = dBm_to_Watts(power_FWD);

            sprintf(myString_FWD, "%.3f", average_voltage_FWD);
            sprintf(myString2_FWD, "%.3f", power_FWD);

//     	    ssd1306_Fill(Black);
//     	    ssd1306_SetCursor(5, 5);
//     	    ssd1306_WriteString(myString_FWD, Font_7x10, White);
//     	    ssd1306_SetCursor(5, 25);
//     	    ssd1306_WriteString(myString2_FWD, Font_7x10, White);
//     	    ssd1306_UpdateScreen();

            error_count_FWR = 0;  // Resetujemy licznik po akceptacji wartości
        }
    }
    if (is_valid_sample(previous_sample_REF, new_sample_REF)) {
            // Jeśli próbka jest poprawna, resetujemy licznik błędów
            error_count_REF = 0;

            // Zapisujemy nową próbkę w buforze kołowym (nadpisujemy najstarszą)
            samples_REF[sample_index_REF] = new_sample_REF;
            sample_index_REF = (sample_index_REF + 1) % ILOSC_PROBEK;  // Przesuwamy wskaźnik cyklicznie

            // Aktualizujemy poprzednią próbkę
            previous_sample_REF = new_sample_REF;

            // Możemy teraz przetworzyć średnią z próbek
            average_voltage_REF = calculate_average_REF();

            power_REF = average_voltage_REF / -0.01225f + 22.877551f;
            power_REF += attenuation_REF;
            powerWatts_REF = dBm_to_Watts(power_REF);

            sprintf(myString_REF, "%.3f", average_voltage_REF);
            sprintf(myString2_REF, "%.3f", power_REF);

        } else {
            // W przypadku błędnej próbki
            error_count_REF++;

            if (error_count_REF >= ERROR_THRESHOLD) {
                // Akceptujemy wartości mimo błędów, jeśli licznik błędów przekroczył próg
                samples_REF[sample_index_REF] = new_sample_REF;
                sample_index_REF = (sample_index_REF + 1) % ILOSC_PROBEK;

                // Aktualizujemy poprzednią próbkę
                previous_sample_REF = new_sample_REF;

                average_voltage_REF = calculate_average_REF();

                power_REF = average_voltage_REF / -0.01225f + 22.877551f;
                power_REF += attenuation_REF;
                powerWatts_REF = dBm_to_Watts(power_REF);

                sprintf(myString_REF, "%.3f", average_voltage_REF);
                sprintf(myString2_REF, "%.3f", power_REF);

                error_count_REF = 0;  // Resetujemy licznik po akceptacji wartości
            }
        }
    WFS = calculateWFS(powerWatts_FWD, powerWatts_REF);
    sprintf(myString_WFS, "%.3f", WFS);
    // Wyświetlanie na ekranie
//    ssd1306_Fill(Black);
//    ssd1306_SetCursor(5, 16);
//    ssd1306_WriteString(myString_FWD, Font_7x10, White);
//    ssd1306_SetCursor(5, 35);
//    ssd1306_WriteString(myString2_FWD, Font_7x10, White);
//    ssd1306_SetCursor(60, 16);
//    ssd1306_WriteString(myString_REF, Font_7x10, White);
//    ssd1306_SetCursor(60, 35);
//    ssd1306_WriteString(myString2_REF, Font_7x10, White);
//    ssd1306_UpdateScreen();
}



// Sprawdzenie, czy różnica między dwiema próbkami nie przekracza TRESHOLD
int is_valid_sample(float sample1, float sample2) {
    float difference = fabs(sample1 - sample2) / sample1;
    return difference <= MAX_DIFFERENCE;
}

// Funkcja do obliczania średniej z trzech próbek
float calculate_average_FWR() {
    float sum = 0.0f;

    for (int i = 0; i < ILOSC_PROBEK; i++) {
        sum += samples_FWR[i];
    }

    return sum / ILOSC_PROBEK;  // Zwracamy średnią z trzech próbek
}

float calculate_average_REF() {
    float sum = 0.0f;

    for (int i = 0; i < ILOSC_PROBEK; i++) {
        sum += samples_REF[i];
    }

    return sum / ILOSC_PROBEK;  // Zwracamy średnią z trzech próbek
}

// Callback dla przerwania EXTI
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == Comp_input_Pin) {
    	adcValue_FWR = value_FWR[0];  // Odczytaj wartość z bufora DMA
    	adcValue_REF = value_REF[0];
        comparator = 1;
    }
}

// Funkcja przeliczająca dBm na Waty
float dBm_to_Watts(float dBm) {
    return pow(10, dBm / 10.0f) * 0.001f;
}

float calculateWFS(float power_incident_watts, float power_reflected_watts) {

    // Oblicz amplitudy fali (pierwiastek z mocy)
    float amplitude_incident = sqrt(power_incident_watts);
    float amplitude_reflected = sqrt(power_reflected_watts);

    // Oblicz współczynnik refleksji Gamma
    float gamma = amplitude_reflected / amplitude_incident;

    // Oblicz WFS
    float wfs = (1 + gamma) / (1 - gamma);

    return wfs;
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
