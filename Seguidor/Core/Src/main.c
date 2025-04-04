/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SENSORS 8  // Número de sensores

#define MOTOR_L_IN1 GPIO_PIN_10  // Ajuste conforme sua conexão PB10
#define MOTOR_L_IN2 GPIO_PIN_5   // PB5
#define MOTOR_L_PWM GPIO_PIN_4  // PWM para o motor esquerdo PB4

#define MOTOR_R_IN1 GPIO_PIN_10 //PA10
#define MOTOR_R_IN2 GPIO_PIN_8  //PA8
#define MOTOR_R_PWM GPIO_PIN_5  // PWM para o motor direito PC7

#define MOTOR_L_PORT GPIOB  // Ajuste a porta correta
#define MOTOR_R_PORT GPIOA  // Ajuste a porta correta

#define PWM_MAX 100  // Definir velocidade máxima (0-100%)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;
uint8_t sensor_values[8];

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM3_Init(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
// Mapeamento dos pinos no STM32
GPIO_TypeDef *SENSOR_PORT[NUM_SENSORS] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOC, GPIOC, GPIOB}; //PA0 PA1 PA4 PA6 PA7 PC4 PC5 PB1
uint16_t SENSOR_PIN[NUM_SENSORS] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_4, GPIO_PIN_6, GPIO_PIN_7,
									GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_1};

uint8_t sensor_values[NUM_SENSORS];  // Armazena os valores lidos
int sensor_positions[NUM_SENSORS] = {-3, -2, -1, 0, 0, 1, 2, 3};  // Posições relativas dos sensores


//variáveis globais para cálculo de PID
float Kp = 25.0;//Comece com Ki = 0. Mexa só no Kp e Kd primeiro.
float Ki = 0.0; //Aumente Kp até o robô reagir bem.
float Kd = 15.0;//Aumente Kd se ele estiver oscilando demais.
				//Só adicione Ki se ele estiver desviando com o tempo.
float error = 0;
float previous_error = 0;
float integral = 0;
float target_position = 0.0;  // Alvo do centroide (pode ajustar conforme seu mapeamento)


void read_sensors();
void print_sensor_values();
float calculate_centroid();

int get_position();
void adjust_motors();
float compute_pid(float position);
void motor_init();
void set_motor_speeds(float correction);
void motor_set_speed(int left_speed, int right_speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // Motor esquerdo
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // Motor direito


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();

  void motor_init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  read_sensors();

	  /*
	  	  //testar sensores
	  print_sensor_values();
	  HAL_Delay(100);
	  */
		HAL_GPIO_WritePin(MOTOR_L_PORT, MOTOR_L_IN1, GPIO_PIN_SET);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_MAX);  // Ajuste o Timer correto
	  ///*
	  	  //testar centroide
	  float centroid = calculate_centroid();
	  char msg[50];
	  sprintf(msg, "Centroide: %.2f\r\n", centroid);
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	  HAL_Delay(100);
	  //*/


//	  float position = calculate_centroid();       // posição da linha
	//  float correction = compute_pid(position);    // calcular PID
	  //set_motor_speeds(correction);                // aplicar correção no motor
	  HAL_Delay(10);


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_5, GPIO_PIN_RESET);


  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//LD2_GPIO_Port

  /*Configure GPIO pins : PA0 PA1 PA4 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void read_sensors() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_values[i] = HAL_GPIO_ReadPin(SENSOR_PORT[i], SENSOR_PIN[i]);
    }
}
void print_sensor_values() {
    char msg[50];
    sprintf(msg, "Sensores: %d %d %d %d %d %d %d %d\r\n",
            sensor_values[0], sensor_values[1], sensor_values[2], sensor_values[3], //sensores 2 e 3 sempre "1"
            sensor_values[4], sensor_values[5], sensor_values[6], sensor_values[7]); //sensores 6 e 7 sempre "0"
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}
int get_position() {
    int position = 0;
    int weight = 0;
    int sum = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensor_values[i] == 1) {  // Linha preta detectada
            position += i * 1000;  // Peso proporcional
            weight++;
        }
    }

    if (weight > 0) {
        return position / weight;  // Média ponderada
    } else {
        return -1;  // Nenhum sensor detectou linha
    }
}
void adjust_motors() {
    int pos = get_position();

    if (pos == -1) {
        // Linha perdida -> Fazer curva ou girar
    } else if (pos < 300) {
        // Linha à esquerda -> Girar à esquerda
    } else if (pos > 400) {
        // Linha à direita -> Girar à direita
    } else {
        // Linha centralizada -> Seguir reto
    }
}

float calculate_centroid() {
    int sum_weights = 0;
    int sum_values = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensor_values[i] == 0) {  // Sensor detecta preto (linha)
            sum_weights += sensor_positions[i];
            sum_values++;
        }
    }

    if (sum_values == 0) {
        return -1;  // Nenhum sensor detectou a linha (linha perdida)
    }

    return (float)sum_weights / sum_values;  // Centroide normalizado
}

float compute_pid(float position) {
    if (isnan(position)) {
        return 0;  // Linha perdida → nenhuma correção
    }

    error = position - target_position;
    integral += error;
    float derivative = error - previous_error;



    float correction = Kp * error + Ki * integral + Kd * derivative;

    previous_error = error;

    return correction;
}

void motor_init() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configura pinos de direção como saída
    GPIO_InitStruct.Pin = MOTOR_L_IN1 | MOTOR_L_IN2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_L_PORT, &GPIO_InitStruct);

    // Configura pinos de direção como saída
	GPIO_InitStruct.Pin = MOTOR_R_IN1 | MOTOR_R_IN2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(MOTOR_R_PORT, &GPIO_InitStruct);

    // Configura pinos PWM como saída (opcional: configurar como PWM via Timer)
    GPIO_InitStruct.Pin = MOTOR_L_PWM | MOTOR_R_PWM;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // Se usar Timer PWM
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTOR_L_PORT, &GPIO_InitStruct);
}
void set_motor_speeds(float correction) {
    int base_speed = 50;  // Velocidade padrão (pode ajustar)
    int left_speed = base_speed + correction;
    int right_speed = base_speed - correction;

    // Saturação (evita valores negativos ou muito altos)
    if (left_speed > 100) left_speed = 100;
    if (right_speed > 100) right_speed = 100;
    if (left_speed < 0) left_speed = 0;
    if (right_speed < 0) right_speed = 0;

    motor_set_speed(left_speed, right_speed);  // Sua função para controlar os motores
}
void motor_set_speed(int left_speed, int right_speed) {
	// Limitar valores entre -100 e 100
	if (left_speed > 100) left_speed = 100;
	if (right_speed > 100) right_speed = 100;
	if (left_speed < -100) left_speed = -100;
	if (right_speed < -100) right_speed = -100;

	// Definir direção e PWM do motor esquerdo
	if (left_speed >= 0) {
		HAL_GPIO_WritePin(MOTOR_L_PORT, MOTOR_L_IN1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_L_PORT, MOTOR_L_IN2, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(MOTOR_L_PORT, MOTOR_L_IN1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_L_PORT, MOTOR_L_IN2, GPIO_PIN_SET);
		left_speed = -left_speed;  // Inverter valor para PWM
	}
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (left_speed * PWM_MAX) / 100);  // Ajuste o Timer correto

	// Definir direção e PWM do motor direito
	if (right_speed >= 0) {
		HAL_GPIO_WritePin(MOTOR_R_PORT, MOTOR_R_IN1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_R_PORT, MOTOR_R_IN2, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(MOTOR_R_PORT, MOTOR_R_IN1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_R_PORT, MOTOR_R_IN2, GPIO_PIN_SET);
		right_speed = -right_speed;
	}
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (right_speed * PWM_MAX) / 100);  // Ajuste o Timer correto
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
