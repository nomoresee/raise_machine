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
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "headfile.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define POS_REDUCTION_NUM          1.5f//输出端角度
#define POS_REDUCTION_DEN          10.0f//电机端角度
#define POS_TEST_OUTPUT_FWD_RAD    1000.0f
#define POS_TEST_OUTPUT_HOME_RAD   0.0f//输出端回零角度
#define POS_TEST_OUTPUT_VEL_LIMIT  40.0f//输出端速度限制
#define POS_TEST_OUTPUT_ACC        8.0f//输出端加速度
#define POS_TEST_FWD_RAD           (POS_TEST_OUTPUT_FWD_RAD * POS_REDUCTION_DEN / POS_REDUCTION_NUM)
#define POS_TEST_HOME_RAD          (POS_TEST_OUTPUT_HOME_RAD * POS_REDUCTION_DEN / POS_REDUCTION_NUM)
#define POS_TEST_VEL_LIMIT         (POS_TEST_OUTPUT_VEL_LIMIT * POS_REDUCTION_DEN / POS_REDUCTION_NUM)
#define POS_ZERO_WAIT_MS           500U
#define POS_HOLD_TIME_MS           1000U
#define POS_PROFILE_POS_EPS        0.01f
#define POS_PROFILE_DT_MAX         0.05f//防止某次程序卡了一下，比如过了 1 秒才进来，位置一下跳太多。现在最大按 0.05s 算
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef enum
{
    POS_TEST_SAVE_ZERO = 0,//保存当前位置为零点
    POS_TEST_WAIT_ZERO,//等零点保存完成
    POS_TEST_GO_1000RAD,//从 0 往 1000rad 走
    POS_TEST_HOLD_1000RAD,//到 1000rad 后停 1 秒
    POS_TEST_BACK_ZERO,//从 1000rad 回 0
    POS_TEST_HOLD_ZERO//到 0 后停 1 秒，然后继续去 1000rad
} pos_test_state_e;

uint32_t print_tick = 0;//控制串口多久打印一次，现在是 100ms 打印一次
uint32_t pos_test_tick = 0;//状态机计时用，比如停 1 秒、等 500ms
uint32_t pos_profile_tick = 0;//梯形曲线每次更新时间用
float pos_output_ref = POS_TEST_OUTPUT_HOME_RAD;//当前规划出来的“输出端目标位置”
float vel_output_ref = 0.0f;//当前规划出来的“输出端目标速度”
pos_test_state_e pos_test_state = POS_TEST_SAVE_ZERO;//当前状态机状态

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void pos_1000_back_update(void);
static void pos_profile_start(uint32_t now);
static uint8_t pos_profile_update(float target_output_rad, uint32_t now);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM3) {
    dm_motor_ctrl_send(&hfdcan1, &motor[Motor1]);

    // dm_motor_ctrl_send(&hfdcan1, &motor[Motor2]);
	}
}

static float pos_absf(float value)//绝对值
{
    return (value >= 0.0f) ? value : -value;
}

static void pos_apply_output_ref(void)//计算目标速度和位置，并发送给电机
{
    motor[Motor1].ctrl.pos_set = pos_output_ref * POS_REDUCTION_DEN / POS_REDUCTION_NUM;
    motor[Motor1].ctrl.vel_set = pos_absf(vel_output_ref) * POS_REDUCTION_DEN / POS_REDUCTION_NUM;
}

static void pos_hold_output(float output_pos_rad)//停在某个位置不动，速度设为 0，位置设为目标位置，并发送给电机
{
    pos_output_ref = output_pos_rad;
    vel_output_ref = 0.0f;
    pos_apply_output_ref();
}

static void pos_profile_start(uint32_t now)//规划速度清零，然后记录当前时间
{
    vel_output_ref = 0.0f;
    pos_profile_tick = now;
    pos_apply_output_ref();
}

static uint8_t pos_profile_update(float target_output_rad, uint32_t now)
{
    float dt = (now - pos_profile_tick) / 1000.0f;//第一步，算距离上次更新过去多久
    float error;
    float distance;
    float direction;
    float speed;
    float stop_distance;
    float step;

    if (dt <= 0.0f)
    {
        pos_apply_output_ref();
        return 0;
    }

    if (dt > POS_PROFILE_DT_MAX)
    {
        dt = POS_PROFILE_DT_MAX;
    }

    pos_profile_tick = now;

    error = target_output_rad - pos_output_ref;//第二步，算当前位置和目标位置的误差
    distance = pos_absf(error);
    speed = pos_absf(vel_output_ref);

    if ((distance <= POS_PROFILE_POS_EPS) && (speed <= POS_PROFILE_POS_EPS))
    {
        pos_hold_output(target_output_rad);
        return 1;
    }

    direction = (error >= 0.0f) ? 1.0f : -1.0f;//第三步，算出当前位置和目标位置的方向
    stop_distance = (speed * speed) / (2.0f * POS_TEST_OUTPUT_ACC);//判断什么时候该减速

    if (stop_distance >= distance)
    {
        speed -= POS_TEST_OUTPUT_ACC * dt;
    }
    else
    {
        speed += POS_TEST_OUTPUT_ACC * dt;
    }

    if (speed > POS_TEST_OUTPUT_VEL_LIMIT)
    {
        speed = POS_TEST_OUTPUT_VEL_LIMIT;
    }

    if (speed < 0.0f)
    {
        speed = 0.0f;
    }

    step = speed * dt;
    if (step >= distance)
    {
        pos_hold_output(target_output_rad);
        return 1;
    }

    vel_output_ref = direction * speed;
    pos_output_ref += vel_output_ref * dt;
    pos_apply_output_ref();
    return 0;
}

static void pos_1000_back_update(void)
{
    uint32_t now = HAL_GetTick();

    switch (pos_test_state)
    {
        case POS_TEST_SAVE_ZERO:
            pos_hold_output(POS_TEST_OUTPUT_HOME_RAD);
            save_pos_zero(&hfdcan1, motor[Motor1].id, POS_MODE);
            pos_test_tick = now;
            pos_test_state = POS_TEST_WAIT_ZERO;
            break;

        case POS_TEST_WAIT_ZERO:
            pos_hold_output(POS_TEST_OUTPUT_HOME_RAD);
            if (now - pos_test_tick >= POS_ZERO_WAIT_MS)
            {
                pos_test_tick = now;
                pos_profile_start(now);
                pos_test_state = POS_TEST_GO_1000RAD;
            }
            break;

        case POS_TEST_GO_1000RAD:
            if (pos_profile_update(POS_TEST_OUTPUT_FWD_RAD, now))
            {
                pos_test_tick = now;
                pos_test_state = POS_TEST_HOLD_1000RAD;
            }
            break;

        case POS_TEST_HOLD_1000RAD:
            pos_hold_output(POS_TEST_OUTPUT_FWD_RAD);
            if (now - pos_test_tick >= POS_HOLD_TIME_MS)
            {
                pos_test_tick = now;
                pos_profile_start(now);
                pos_test_state = POS_TEST_BACK_ZERO;
            }
            break;

        case POS_TEST_BACK_ZERO:
            if (pos_profile_update(POS_TEST_OUTPUT_HOME_RAD, now))
            {
                pos_test_tick = now;
                pos_test_state = POS_TEST_HOLD_ZERO;
            }
            break;

        case POS_TEST_HOLD_ZERO:
            pos_hold_output(POS_TEST_OUTPUT_HOME_RAD);
            if (now - pos_test_tick >= POS_HOLD_TIME_MS)
            {
                pos_test_tick = now;
                pos_profile_start(now);
                pos_test_state = POS_TEST_GO_1000RAD;
            }
            break;

        default:
            pos_hold_output(POS_TEST_OUTPUT_HOME_RAD);
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
  MX_FDCAN1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	power(1);
  HAL_Delay(1000);
	delay_init(480);
	bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);
//	bsp_fdcan_set_baud(&hfdcan1, CAN_FD_BRS, CAN_BR_5M);
	bsp_can_init();
	dm_motor_init();
	HAL_Delay(10);

	motor[Motor1].ctrl.mode = pos_mode;
	motor[Motor1].ctrl.pos_set = POS_TEST_HOME_RAD;
	motor[Motor1].ctrl.vel_set = POS_TEST_VEL_LIMIT;

	// Write control mode.
	write_motor_data(motor[Motor1].id, 10, pos_mode, 0, 0, 0);
	HAL_Delay(50);
  save_motor_data(motor[Motor1].id, 10);
	HAL_Delay(50);
  //write_motor_data(motor[Motor2].id, 10, spd_mode, 0, 0, 0);
	//HAL_Delay(50);
 // save_motor_data(motor[Motor2].id, 10);
	//HAL_Delay(50);
//	write_motor_data(motor[Motor1].id, 35, CAN_BR_1M, 0, 0, 0);
//	write_motor_data(motor[Motor1].id, 0x3C, 0, 0, 0, 0);
//	HAL_Delay(100);
//	read_motor_data(motor[Motor1].id, RID_CAN_BR); 
//	dm_motor_disable(&hfdcan1, &motor[Motor1]);
//	HAL_Delay(100);

  //清除错误标志，准备使能电机
  dm_motor_clear_err(&hfdcan1, &motor[Motor1]);
  HAL_Delay(100);
  //dm_motor_clear_err(&hfdcan1, &motor[Motor2]);
  //HAL_Delay(100);
	dm_motor_enable(&hfdcan1, &motor[Motor1]);
	HAL_Delay(100);
	//dm_motor_enable(&hfdcan1, &motor[Motor2]);
	//HAL_Delay(100);

	HAL_TIM_Base_Start_IT(&htim3);
  //save_pos_zero(&hfdcan1, motor[Motor1].id,POS_MODE);
//	read_all_motor_data(&motor[Motor1]);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    pos_1000_back_update();

    if (HAL_GetTick() - print_tick >= 100)
    {
        print_tick = HAL_GetTick();
        //read_motor_data(motor[Motor1].id, RID_X_OUT);
        printf("%d,%.2f,%.2f,%.2f,%.2f\r\n",
               pos_test_state,
               motor[Motor1].ctrl.pos_set * POS_REDUCTION_NUM / POS_REDUCTION_DEN,
               motor[Motor1].ctrl.pos_set,
               motor[Motor1].para.pos,
               motor[Motor1].para.vel);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)//重定向printf函数到串口
{
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart1, &c, 1, HAL_MAX_DELAY);
    return ch;
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
