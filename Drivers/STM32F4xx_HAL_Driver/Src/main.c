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
#include "TIMER_TIMEOUT.h"
#include "WT901C.h"
#include "PS4_ESP.h"
#include "NEXTION_HMI.h"
#include "DRIVER_PID_AML.h"
#include "DRIVER_PID_TGC.h"
#include "OMNI_3_FIELD_KIN.h"
#include "SWERVE_4_KINEMATIC.h"

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/*Motor driver variables*/
DRIVER_PID_TGC DRV_TGC_1;
DRIVER_PID_TGC DRV_TGC_2;
DRIVER_PID_TGC DRV_TGC_3;

/*Robot control variables*/
ORb Omni_3_Bot;
PID_TypeDef Omega_PID;
/*IMU variables*/
WT901C IMU;

/*Timeout flag variables*/
Task_Timeout Task_TO[TASK_NUMS];
// Task_TO[0] : IMU timeout
// Task_TO[1] : PS4_timeout
// Task_TO[2] : Mainloop_timeout

/*PS4 data variables*/
PS4_DATA PS4_Dat;
volatile BS Button_State = BUTTON_NONE;

/*Nextion HMI variables*/
Nextion nextion;

NexComp imu_val;
NexComp Encoder_X_val;
NexComp Encoder_Y_val;
NexComp Lazer_X_val;
NexComp Lazer_Y_val;

NexComp up_ps4_val;
NexComp down_ps4_val;
NexComp right_ps4_val;
NexComp left_ps4_val;
NexComp triag_ps4_val;
NexComp cross_ps4_val;
NexComp circle_ps4_val;
NexComp square_ps4_val;
NexComp r1_ps4_val;
NexComp l1_ps4_val;
NexComp r3_ps4_val;
NexComp l3_ps4_val;
NexComp opt_ps4_val;
NexComp share_ps4_val;
NexComp ps_ps4_val;
NexComp kp_ps4_val;

NexComp rx_bar;
NexComp ry_bar;
NexComp lx_bar;
NexComp ly_bar;
NexComp r2_bar;
NexComp l2_bar;

NexComp ps4_fault;
NexComp imu_fault;

NexComp test_ps4_button;
NexComp home_button;
NexComp robot_home_button;

typedef enum
{
  HOME_PAGE,
  PS4_PAGE
} PageState;
PageState Page_State = HOME_PAGE;

/*User variables*/
float u1, u2, u3;

uint8_t home_rb_Button;
int32_t x_encoder, y_encoder;
uint8_t Is_Neg_Heading = 0;
int16_t Closet_Angles[5];
uint8_t IMU_Fix = 0;
uint8_t Tim14_Counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM12_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Buzzer_Beep(uint32_t onTime, uint32_t offTime, uint8_t count)
{
  for (uint8_t i = 0; i < count; i++)
  {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    HAL_Delay(onTime);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    if (i < count - 1)
    {
      HAL_Delay(offTime);
    }
  }
}

/*Timer overflow interrupt slot*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) // x_ms handle khong nho bao nhieu :))
  {
    WT901C_Angle_Request(&IMU);
  }
  if (htim->Instance == TIM14) // 1ms handle
  {
    Timer_Timeout_Check(Task_TO);
    PS4_UART_Req();
  }
  if (htim->Instance == TIM13)
  {
    OmniRobot_Field_Control(&Omni_3_Bot, &PS4_Dat, IMU.Yaw, Is_Neg_Heading);
  }
}
/*UART IDLE interrupt slot*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2)
  {
    WT901C_UART_Rx_IDLE_Hanlde(&IMU);
    Find_Closet_Angle(IMU.Yaw, Closet_Angles);
    Timer_Timeout_Reset(&Task_TO[0]);
  }
  if (huart->Instance == USART3)
  {
    PS4_UART_Rx_IDLE_Handle();
    Timer_Timeout_Reset(&Task_TO[1]);
  }
}
/*UART Rx interrupt slot*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  NextionUpdate(huart, &nextion);
}

/*init function & user callback for Nextion HMI*/

void Test_PS4_Button_Handle(void)
{
  Page_State = PS4_PAGE;
}

void Home_Button_Handle(void)
{
  Page_State = HOME_PAGE;
}

void Home_Robot_Button_Handle(void)
{
  home_rb_Button = 1;
}

void Init_Nextion_Comp(void)
{
  /////////////////////////////home-page/////////////////////////////
  NextionAddComp(&nextion, &imu_val, "n0", 0, 10, NULL, NULL);
  NextionAddComp(&nextion, &Encoder_X_val, "n1", 0, 11, NULL, NULL);
  NextionAddComp(&nextion, &Encoder_Y_val, "n2", 0, 12, NULL, NULL);
  NextionAddComp(&nextion, &Lazer_X_val, "n3", 0, 13, NULL, NULL);
  NextionAddComp(&nextion, &Lazer_Y_val, "n4", 0, 14, NULL, NULL);

  NextionAddComp(&nextion, &ps4_fault, "va1", 0, 20, NULL, NULL);
  NextionAddComp(&nextion, &imu_fault, "va0", 0, 19, NULL, NULL);

  NextionAddComp(&nextion, &test_ps4_button, "ps4", 0, 16, NULL, Test_PS4_Button_Handle);
  NextionAddComp(&nextion, &robot_home_button, "rb_home", 0, 9, NULL, Home_Robot_Button_Handle);
  /////////////////////////////ps4_page//////////////////////////////
  NextionAddComp(&nextion, &up_ps4_val, "va0", 1, 33, NULL, NULL);
  NextionAddComp(&nextion, &down_ps4_val, "va1", 1, 34, NULL, NULL);
  NextionAddComp(&nextion, &right_ps4_val, "va2", 1, 35, NULL, NULL);
  NextionAddComp(&nextion, &left_ps4_val, "va3", 1, 36, NULL, NULL);
  NextionAddComp(&nextion, &triag_ps4_val, "va4", 1, 37, NULL, NULL);
  NextionAddComp(&nextion, &cross_ps4_val, "va5", 1, 38, NULL, NULL);
  NextionAddComp(&nextion, &circle_ps4_val, "va6", 1, 39, NULL, NULL);
  NextionAddComp(&nextion, &square_ps4_val, "va7", 1, 40, NULL, NULL);
  NextionAddComp(&nextion, &r1_ps4_val, "va8", 1, 41, NULL, NULL);
  NextionAddComp(&nextion, &l1_ps4_val, "va9", 1, 42, NULL, NULL);
  NextionAddComp(&nextion, &r3_ps4_val, "va10", 1, 43, NULL, NULL);
  NextionAddComp(&nextion, &l3_ps4_val, "va11", 1, 44, NULL, NULL);
  NextionAddComp(&nextion, &opt_ps4_val, "va12", 1, 45, NULL, NULL);
  NextionAddComp(&nextion, &share_ps4_val, "va13", 1, 46, NULL, NULL);
  NextionAddComp(&nextion, &ps_ps4_val, "va14", 1, 47, NULL, NULL);
  NextionAddComp(&nextion, &kp_ps4_val, "va15", 1, 48, NULL, NULL);

  NextionAddComp(&nextion, &rx_bar, "j0", 1, 4, NULL, NULL);
  NextionAddComp(&nextion, &ry_bar, "j1", 1, 5, NULL, NULL);
  NextionAddComp(&nextion, &lx_bar, "j2", 1, 6, NULL, NULL);
  NextionAddComp(&nextion, &ly_bar, "j3", 1, 7, NULL, NULL);
  NextionAddComp(&nextion, &r2_bar, "j4", 1, 8, NULL, NULL);
  NextionAddComp(&nextion, &l2_bar, "j5", 1, 9, NULL, NULL);

  NextionAddComp(&nextion, &home_button, "home", 1, 3, NULL, Home_Button_Handle);
}

void Home_Page_Nextion_Print(void)
{
  x_encoder = __HAL_TIM_GetCounter(&htim2);
  y_encoder = __HAL_TIM_GetCounter(&htim5);
  NextionSetVal(&nextion, &imu_val, IMU.Yaw);
  NextionSetVal(&nextion, &Encoder_X_val, x_encoder / 4);
  NextionSetVal(&nextion, &Encoder_Y_val, y_encoder / 4);
  if (Task_TO[1].Time_Out_Flag)
  {
    NextionSetVal(&nextion, &ps4_fault, 0);
  }
  else
  {
    NextionSetVal(&nextion, &ps4_fault, 1);
  }

  if (Task_TO[0].Time_Out_Flag)
  {
    NextionSetVal(&nextion, &imu_fault, 0);
  }
  else
  {
    NextionSetVal(&nextion, &imu_fault, 1);
  }
}

void PS4_Page_Nextion_Print(void)
{
  NextionSetVal(&nextion, &up_ps4_val, Button_State == BUTTON_UP ? 1 : 0);
  NextionSetVal(&nextion, &down_ps4_val, Button_State == BUTTON_DOWN ? 1 : 0);
  NextionSetVal(&nextion, &right_ps4_val, Button_State == BUTTON_RIGHT ? 1 : 0);
  NextionSetVal(&nextion, &left_ps4_val, Button_State == BUTTON_LEFT ? 1 : 0);
  NextionSetVal(&nextion, &triag_ps4_val, Button_State == BUTTON_TRIANGLE ? 1 : 0);
  NextionSetVal(&nextion, &cross_ps4_val, Button_State == BUTTON_CROSS ? 1 : 0);
  NextionSetVal(&nextion, &circle_ps4_val, Button_State == BUTTON_CIRCLE ? 1 : 0);
  NextionSetVal(&nextion, &square_ps4_val, Button_State == BUTTON_SQUARE ? 1 : 0);
  NextionSetVal(&nextion, &r1_ps4_val, Button_State == BUTTON_R1 ? 1 : 0);
  NextionSetVal(&nextion, &l1_ps4_val, Button_State == BUTTON_L1 ? 1 : 0);
  NextionSetVal(&nextion, &r3_ps4_val, Button_State == BUTTON_R3 ? 1 : 0);
  NextionSetVal(&nextion, &l3_ps4_val, Button_State == BUTTON_L3 ? 1 : 0);
  NextionSetVal(&nextion, &opt_ps4_val, Button_State == BUTTON_OPTIONS ? 1 : 0);
  NextionSetVal(&nextion, &share_ps4_val, Button_State == BUTTON_SHARE ? 1 : 0);
  NextionSetVal(&nextion, &ps_ps4_val, Button_State == BUTTON_PSBUTTON ? 1 : 0);
  NextionSetVal(&nextion, &kp_ps4_val, Button_State == BUTTON_TOUCHPAD ? 1 : 0);

  NextionSetVal(&nextion, &rx_bar, (uint8_t)((128 + PS4_Dat.r_stick_x) / 2.55f));
  NextionSetVal(&nextion, &ry_bar, (uint8_t)((128 + PS4_Dat.r_stick_y) / 2.55f));
  NextionSetVal(&nextion, &lx_bar, (uint8_t)((128 + PS4_Dat.l_stick_x) / 2.55f));
  NextionSetVal(&nextion, &ly_bar, (uint8_t)((128 + PS4_Dat.l_stick_y) / 2.55f));
  NextionSetVal(&nextion, &r2_bar, (uint8_t)((PS4_Dat.r2_analog) / 2.55f));
  NextionSetVal(&nextion, &l2_bar, (uint8_t)((PS4_Dat.l2_analog) / 2.55f));
}

/*PS4 button macros handles*/

void Wait_Button_Release(uint16_t button)
{
  while (Button_State == button)
  {
  }
}

void Process_Button_Input(BS button)
{
  switch (button)
  {
  case BUTTON_OPTIONS: // Reset IMU Z angle
    Wait_Button_Release(BUTTON_OPTIONS);
    home_rb_Button = 0;
    __HAL_TIM_SetCounter(&htim2, 0);
    __HAL_TIM_SetCounter(&htim5, 0);
    HAL_TIM_Base_Stop_IT(&htim1);
    WT901C_Reset_Angles(&IMU);
    HAL_TIM_Base_Start_IT(&htim1);
    Buzzer_Beep(60, 10, 2);
    break;

  case BUTTON_L1: // Toggle heading direction
    Wait_Button_Release(BUTTON_L1);
    Is_Neg_Heading = !Is_Neg_Heading;
    Buzzer_Beep(80, 10, 2);
    break;

  case BUTTON_PSBUTTON: // Toggle yaw fix mode
    Wait_Button_Release(BUTTON_PSBUTTON);
    Omni_3_Bot.is_yaw_fix = !Omni_3_Bot.is_yaw_fix;
    Buzzer_Beep(100, 10, 1);
    break;

  case BUTTON_TRIANGLE: // Set fix angle to 0
    Wait_Button_Release(BUTTON_TRIANGLE);
    Omni_3_Bot.fix_angle = 0;
    break;

  case BUTTON_CIRCLE: // Set fix angle to -90
    Wait_Button_Release(BUTTON_CIRCLE);
    Omni_3_Bot.fix_angle = -90;
    break;

  case BUTTON_CROSS: // Set fix angle to 180
    Wait_Button_Release(BUTTON_CROSS);
    Omni_3_Bot.fix_angle = 180;
    break;

  case BUTTON_SQUARE: // Set fix angle to 90
    Wait_Button_Release(BUTTON_SQUARE);
    Omni_3_Bot.fix_angle = 90;
    break;

  default:
    break;
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
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_TIM12_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);

  PS4_Init(&huart3);
  WT901C_Init(&IMU, &huart2);
  PID_TGC_Init(&huart4);
  NextionInit(&nextion, &huart1);
  OmniRobot_Init(&Omni_3_Bot, Robot_Max_Speed, Robot_Max_Omega);

  /*Assign id for driver mode UART*/
  Assign_PID_TGC_Id(&DRV_TGC_1, 0x01);
  Assign_PID_TGC_Id(&DRV_TGC_2, 0x02);
  Assign_PID_TGC_Id(&DRV_TGC_3, 0x03);

  /*Calib IMU*/
  WT901C_Reset_Angles(&IMU);
  HAL_Delay(5000); // 5 secs to calib IMU

  /*Init components of HMI*/
  Init_Nextion_Comp();

  /*Set timeout & start timeout for tasks*/
  Timer_Timeout_Init(&Task_TO[0], 1000); // Task for IMU timeout
  Timer_Timeout_Init(&Task_TO[1], 1000); // Task for PS4 timeout
  Timer_Timeout_Init(&Task_TO[2], 1000); // Task for mainloop timeout
  for (uint8_t i = 0; i < 3; i++)
  {
    Task_TO[i].Time_Out_Flag = 1;
  }
  Timer_Timeout_Start(&htim14);
  /*------------------------------------*/

  HAL_TIM_Base_Start_IT(&htim1); // Timer to request IMU angles

  /*Start encoder capture*/
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  /*------------------------------------*/

  /*PID IMU angle fix timer*/
  HAL_TIM_Base_Start_IT(&htim13);
  Buzzer_Beep(80, 10, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Omni_3_Bot.max_speed = (PS4_Dat.r2_analog / 255.0f) * 8 + Robot_Max_Speed;
    Omni_3_Bot.max_omega = (PS4_Dat.r2_analog / 255.0f) * 8 + Robot_Max_Omega;

    Process_Button_Input(Button_State);

    switch (Page_State) // Switch UART data content transmit to HMI
    {
    case HOME_PAGE:
      Home_Page_Nextion_Print();
      break;
    case PS4_PAGE:
      PS4_Page_Nextion_Print();
      break;
    }

    if (Task_TO[1].Time_Out_Flag) // PS4 disconnected, switch to braking mode
    {
      Brake_Motor_TGC(&DRV_TGC_1);
      Brake_Motor_TGC(&DRV_TGC_2);
      Brake_Motor_TGC(&DRV_TGC_3);
    }
    else
    {
      Send_Speed_TGC(&DRV_TGC_1, Omni_3_Bot.u[0]);
      Send_Speed_TGC(&DRV_TGC_2, Omni_3_Bot.u[2]);
      Send_Speed_TGC(&DRV_TGC_3, Omni_3_Bot.u[1]);
    }

    Timer_Timeout_Reset(&Task_TO[2]);
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16800;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
}

/**
 * @brief TIM12 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 167;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
}

/**
 * @brief TIM13 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 27;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 59999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 2;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 55999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
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
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PS2_CS_Pin | PS2_SCK_Pin | PS2_MOSI_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UART_SOFT_TX1_GPIO_Port, UART_SOFT_TX1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PS2_CS_Pin PS2_SCK_Pin PS2_MOSI_Pin */
  GPIO_InitStruct.Pin = PS2_CS_Pin | PS2_SCK_Pin | PS2_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PS2_MISO_Pin */
  GPIO_InitStruct.Pin = PS2_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PS2_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UART_SOFT_RX1_Pin */
  GPIO_InitStruct.Pin = UART_SOFT_RX1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(UART_SOFT_RX1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UART_SOFT_TX1_Pin */
  GPIO_InitStruct.Pin = UART_SOFT_TX1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(UART_SOFT_TX1_GPIO_Port, &GPIO_InitStruct);

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
