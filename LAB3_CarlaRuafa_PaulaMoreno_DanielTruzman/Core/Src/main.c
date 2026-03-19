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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern UART_HandleTypeDef huart2;
#include <stdio.h>
int _write(int file, char *ptr, int len) {
	int i=0;
	for(i=0; i<len; i++)
	HAL_UART_Transmit(&huart2,(uint8_t *)(ptr++),1,1000);
	return len;
}
void waiting(unsigned int delay) {
	unsigned int i;
	for (i=0; i<delay; i++);
}
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
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
unsigned char state = 0;
unsigned char medicion_init = 0;
unsigned char medicion_end = 1;
uint32_t value_adc = 0;
int angle = 0;

unsigned char estado_echo = 0;
unsigned char trig =0;
unsigned short tiempo_inicio = 0;
unsigned short tiempo = 0;
unsigned short distancia = 0;
unsigned char new_distance = 0;

unsigned char auto_step = 0; //counts how many 500ms blocks passed since the beginning of the measurement
unsigned char print_angle_flag = 0;//turns 1 when tell the main loop that the value should be printed

uint32_t pulse_width = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void EXTI15_10_IRQHandler(void){
	if(EXTI->PR & (1 << 13)){
		EXTI->PR = (0x01 << 13);
		state = state + 1;
		if (state >1) state = 0;
		if (medicion_end == 0) {
			printf("Mode will change after finishing the measurement \r\n");
		}
	}
}

void EXTI4_IRQHandler(void){
	// no need to verify EXTI PR as it only triggers this function with EXTI4
	EXTI->PR = (0x01 << 4);
	medicion_init = 1;
	medicion_end = 0;
	GPIOB->ODR |= (1 << 5);//Turn on the led
	TIM4->CCR1 = TIM4->CNT + 500;//set the 500ms
	TIM4->SR &= ~(1 << 1);//clear flags
	TIM4->DIER |= (1 << 1);//activate interruption

}

void leer_potenciometro(void){
	ADC1->CR2 |= 0x40000000;     // iniciar conversión
	while((ADC1->SR & 0x02)==0); // esperar fin
	value_adc = ADC1->DR;        // leer resultado
	angle = ((value_adc * 180)/4095)-90;
}

void TIM4_IRQHandler(void) {
	if ((TIM4->SR & (1 << 1)) != 0){
			TIM4->SR &= ~(1 << 1); // clear flag

			if (state == 0) {
				//Manual mode: turn off led and end interruption
				GPIOB->ODR &= ~(1 << 5); // turn off led
				TIM4->DIER &= ~(1 << 1); // turn of dier
				medicion_end = 1;
			} else {
				//Automatic mode
				auto_step++;

				// 1. Control del parpadeo (Pasos pares: Encendido, Pasos impares: Apagado)
				if (auto_step % 2 != 0) {
					GPIOB->ODR &= ~(1 << 5); // Apagar
				} else {
					GPIOB->ODR |= (1 << 5);  // Encender
				}

				// 2. Cambiar de ángulo cada 4 pasos (4 * 500ms = 2 segundos exactos)
				if (auto_step % 4 == 0) {
					int index = auto_step / 4;
					if (index < 5) {
						angle = -90 + (index * 45); // Siguiente ángulo a medir
					} else {
						angle = 0; // Regresar a la posición central al acabar
					}
					print_angle_flag = 1; // Avisamos al main() para que haga el printf
				}

				// 3. Evaluar si hemos terminado el ciclo (5 ángulos * 4 pasos = 20)
				if (auto_step >= 20) {
					TIM4->DIER &= ~(1 << 1); // Desactivar interrupción
					GPIOB->ODR &= ~(1 << 5); // Asegurar que el LED quede apagado
					medicion_end = 1;        // Liberar el estado para cambiar de modo
				} else {
					// Seguir con la secuencia: programar los siguientes 500ms
					TIM4->CCR1 = TIM4->CNT + 500;
				}
			}
		}
}
void medir(void){
	 GPIOC->BSRR = (1<<8);
	 TIM3 -> CNT = 0;
	 while(TIM3-> CNT < 10);
	 GPIOC->BSRR = (1<<8)<<16;
}

void TIM3_IRQHandler(void){
	if(TIM3->SR & 0x0010)
	    {
	        if(estado_echo==0)
	        {
	            tiempo_inicio = TIM3->CCR4;

	            estado_echo=1;

	            TIM3->CCER |= (1<<13);   // Detect Falling
	        }
	        else
	        {
	            tiempo = TIM3->CCR4 - tiempo_inicio;

	            if(tiempo<0)
	                tiempo+=0xFFFF;

	            estado_echo=0;

	            TIM3->CCER &= ~(1<<13);  //Detect rising

	            distancia = tiempo /58;
	            new_distance = 1;
	        }

	        TIM3->SR=0;
	    }

}

void update_led_distance(unsigned short d){
  //Tturn off the 5 LEDs (PC0 a PC4). We use the bits of Reset from BSRR (del 16 al 20)
  GPIOC ->BSRR = (0X1F << 16);
  //Turn on the LEDs
  if (d <= 80){
    GPIOC ->BSRR |= (1 << 0); //LED1
    GPIOC ->BSRR |= (1 << 1)<<16; //LED2
    GPIOC ->BSRR |= (1 << 2)<<16; //LED3
    GPIOC ->BSRR |= (1 << 3)<<16; //LED4
    GPIOC ->BSRR |= (1 << 4)<<16; //LED5
  }if (d <= 60){
    GPIOC ->BSRR |= (1 << 1); //LED2
    GPIOC ->BSRR |= (1 << 2)<<16; //LED3
	GPIOC ->BSRR |= (1 << 3)<<16; //LED4
	GPIOC ->BSRR |= (1 << 4)<<16; //LED5
  }
  if (d <= 40){
    GPIOC ->BSRR |= (1 << 2); //LED3
	GPIOC ->BSRR |= (1 << 3)<<16; //LED4
	GPIOC ->BSRR |= (1 << 4)<<16; //LED5
  }
  if (d <= 20){
    GPIOC ->BSRR |= (1 << 3); //LED4
	GPIOC ->BSRR |= (1 << 4)<<16; //LED5
  }
  if (d <= 10){
    GPIOC ->BSRR |= (1 << 4); //LED5
  }
}

void servo(int angle){
	if(angle < -90){
		angle = -90;
	}

	if(angle > 90){
		angle = 90;
	}
	pulse_width = 1500 + (angle * 500) / 90;
	TIM2->CCR1 = pulse_width;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	unsigned char prev_state = 0;
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
  MX_ADC_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /*INPUTS: */
  /*PB4 configuration EXTI mode, moder (00)*/
  GPIOB ->MODER &= ~(1 << (4*2 + 1));
  GPIOB ->MODER &= ~(1 << (4*2));
  GPIOB ->PUPDR &= ~(3 << 8); // cleaning any previous configuration
  GPIOB ->PUPDR |= (2 << 8); // Setting button on pull down (10)

  SYSCFG ->EXTICR[1] &= ~(0xF << 0);	//Cleaning any new previous at SYSCFG
  SYSCFG->EXTICR[1] |= (0x01 << 0);		// Enabling EXTI4 for PB4 with 0001
  EXTI ->FTSR &= ~(0x01 << 4);			// Disabling falling edge
  EXTI ->RTSR |= (0x01 << 4);			// Enabling rising edge
  EXTI ->IMR |= (0x01 << 4); 			// Enable EXTI 4
  NVIC ->ISER[0] |= (1 << 10); 	// Enable EXTI4 NVIC

  /*PC13 configuration EXTI mode, moder(00)*/
  GPIOC ->MODER &= ~(1 << (13*2 + 1));
  GPIOC ->MODER &= ~(1 << (13*2));

  SYSCFG ->EXTICR[3] &= ~(0xF << 4);	//Cleaning any new previous at SYSCFG
  SYSCFG->EXTICR[3] |= (0x02 << 4);		// Enabling EXTI13 for PC13 with 0010
  EXTI ->FTSR |= (0x01 << 13);			// Enable falling edge
  EXTI ->RTSR &= ~(0x01 << 13);			// Disable rising edge
  EXTI ->IMR |= (0x01 << 13); 			// Enable EXTI 13
  NVIC ->ISER[1] |= (1 << (40-32)); 	// Enable EXTI15_10 NVIC

  //PA1 as an input analog mode, (11)
  GPIOA->MODER &= ~(3 << (1*2));
  GPIOA->MODER |= (3 << (1*2));
  ADC1->CR1 = 0x00000000;//Res (00) 12 bits, scan mode 0 disbabled, EOCIE 0
  ADC1->SQR1 = 0x00000000; // 1 channel enabled
  ADC1->SQR3 = 1;
  ADC1->CR2 |= (1 << 0);

  /*OUTPUTS: */
  /*PA5 configuration in output mode, moder(01)  */
  GPIOA ->MODER &= ~(1 << (5*2 + 1));
  GPIOA ->MODER |= (1 << (5*2));


  /*PB5 configuration of the external led, moder(01)*/
  GPIOB ->MODER &= ~(1 << (5*2 + 1));
  GPIOB ->MODER |= (1 << (5*2));

  //PC8 as trig output
    GPIOC -> MODER &= ~(3<<(2*8));
    GPIOC -> MODER |= (1<<(2*8));
    GPIOC->BSRR = (1<<8)<<16;

    /*TIMERS:*/
     /*TIM3_CH4 configuration: */
   	GPIOC->MODER &= ~(3 << (2*9)); //PC9 as output
   	GPIOC->MODER |=  (2 << (2*9));

   	GPIOC->AFR[1] &= ~(0xF << 4);
   	GPIOC->AFR[1] |=  (0x2 << 4); //Activating TIM3

   	TIM3->CR1 = 0x0000;
   	TIM3->CR2 = 0x0000;
   	TIM3->SMCR = 0x0000;

   	TIM3->PSC = 31;
   	TIM3->CNT = 0;
   	TIM3->ARR = 0xFFFF;

   	TIM3->CCMR2 = 0x0100;   // CH4 Input capture
   	TIM3->CCER  = 0x1000;   // Enable CH4
   	TIM3->DIER  = 0x0010;   // Interrupt CH4

   	TIM3->CCER &= ~(1<<13); //Starting rising

   	TIM3->CR1 |= 1;
   	TIM3->EGR |= 1;
   	TIM3->SR = 0;

   	NVIC->ISER[0] |= (1 << 29);

  /*TIMER FOR LED (TIM4)*/
  TIM4->CR1 = 0X0000;//turn off the timer so we can configure it
  TIM4->CR2 = 0X0000;//always 0 in this course
  TIM4->SMCR = 0X0000;//always 0 in this course

  TIM4->PSC = 31999;//set the prescaler
  TIM4->CNT = 0;//initialize the counter
  TIM4->ARR = 0xFFFF;//set the max value

  TIM4->CCMR1 = 0x0000;//set channels 1 and 2 in internal mode
  TIM4->CCER = 0x0000;

  TIM4->CR1 |= 0x0001;//starts the timer
  TIM4->EGR |= 0x0001;//UG = 1 -> generate an update event to update all registers
  TIM4->SR = 0;//clear flags

  NVIC->ISER[0] |= (1 << 30);//Enabling TIM4_IRQ at NVIC (position 30)

  /*TIMER 2 */
  GPIOA->MODER &= ~(3 << (2*0)); // PA0 clear
  GPIOA->MODER |=  (2 << (2*0)); // PA0 as Alternate Function

  GPIOA->AFR[0] &= ~(0xF << 0);  // Cleans AF PA0
  GPIOA->AFR[0] |=  (0x1 << 0);  // Activates AF1

  TIM2->CR1 = 0x0000; //turn off
  TIM2->CR2 = 0x0000;
  TIM2->SMCR = 0x0000;

  TIM2->PSC = 31; //set the prescaler (32MHz)
  TIM2->CNT = 0; //clear counter
  TIM2->ARR = 19999; //set max value (20ms period for servo)

  TIM2->CCMR1 = 0x0060; //set CH1 in PWM mode 1 (bits 6:4 to 110)
  TIM2->CCER  = 0x0001; //enable CH1 output
  TIM2->CCR1  = 1500;   //set initial pulse width (1500us = 0 degrees/center)

  TIM2->CR1 |= 0x0001; //start timer
  TIM2->EGR |= 0x0001; //UG = 1 -> generate an update event to update all registers
  TIM2->SR = 0;        //clear flags

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if ((state != prev_state) && (medicion_end == 1)){
		  prev_state = state;
		  if(state == 0){
		  	 printf("Manual mode\r\n");
		  	 GPIOA->BSRR = (1<<5)<<16;
		  }else{
		  	 printf("Automatic mode\r\n");
		  	 GPIOA->BSRR = (1<<5);
		  }
	  }
	  if (state == 0){//Manual mode
		  leer_potenciometro();
		  servo(angle);
		  if(medicion_init){
		  	 medir();
		  	 medicion_init = 0;
		  }
	  }else{
		  if(medicion_init == 1){
			 auto_step = 0;
			 angle = -90;

			 printf("Automatic angle: %d\r\n",angle);
			 servo(angle);
			 medir();
			 medicion_init = 0;
		  }

		  if (print_angle_flag == 1) {
			  print_angle_flag = 0;
			  printf("Automatic angle: %d\r\n",angle);
			  servo(angle);
			  medir();
		  }
	}
	  if(new_distance){
	  		  printf("Distancia = %d cm\r\n", distancia);
	  		  update_led_distance(distancia);
	  		  new_distance = 0;
	  }
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
  /* USER CODE END 3 */


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
