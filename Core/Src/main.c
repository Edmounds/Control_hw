/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  STATE_DARK,
  STATE_BRIGHT
} LightState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_VOLTAGE 20.0f // 系统最大测量电压 (3.3V * 5.965)
#define PWM_PERIOD 2880   // PWM周期计数值
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PI_Controller voltage_pi;        // 电压环PI控制器
float voltage_setpoint = 12.0;   // 电压设定值 12V
float voltage_feedback = 0;      // 电压反馈值
uint16_t pwm_duty = 0;           // PWM占空比
volatile uint16_t adc_value = 0; // ADC转换结果 (由ADC中断更新)
volatile uint8_t adc_ready = 0;  // ADC数据就绪标志
LightState light_state = STATE_DARK;
float adc2_value = 0.0f;

// RMS Calculation Variables
volatile float adc_rms_voltage = 0.0f;
uint64_t adc_sq_sum = 0;
uint16_t adc_sample_cnt = 0;
#define RMS_SAMPLE_COUNT 100
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  OLED_Init();
  // 初始化PI控制器
  // 对应参数: Kp=1.0, Ki=10.0, 输出限幅[0.0, 1.0]
  PI_Init(&voltage_pi, 1.0f, 12.0f, 1.0f, 0.0f);

  // 启动PWM (TIM1)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // 校准ADC (提高精度)
  HAL_ADCEx_Calibration_Start(&hadc1);

  // 启动控制循环定时器 (TIM2)
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    char buf[16];
    char buf2[16];
    // HAL_ADC_Start(&hadc2);
    // if (HAL_ADC_PollForConversion(&hadc2, 100) == HAL_OK)
    // {
    //   adc2_value = (float)HAL_ADC_GetValue(&hadc2) * 3.3f / 4096.0f;
    // }
    // HAL_ADC_Stop(&hadc2);

    // OLED 中央显示ADC1采样电压（稳压反馈）
    // (void)snprintf(buf, sizeof(buf), "V=%-5.2fV", voltage_feedback);
    (void)snprintf(buf, sizeof(buf), "V=%-5.2fV", adc_rms_voltage);
    OLED_ShowString(32, 1, buf, 16);
    // OLED_ShowString(32, 3, buf, 16);

    HAL_Delay(50); 

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief  初始化PI控制器
 * @param  pi: PI控制器结构体指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  max: 输出最大值
 * @param  min: 输出最小值
 * @retval None
 */
void PI_Init(PI_Controller *pi, float kp, float ki, float max, float min)
{
  pi->Kp = kp;
  pi->Ki = ki;
  pi->integral = 0;
  pi->output_max = max;
  pi->output_min = min;
}

/**
 * @brief  PI控制器更新计算
 * @param  pi: PI控制器结构体指针
 * @param  setpoint: 设定值
 * @param  feedback: 反馈值
 * @retval PI控制器输出值
 */
/**
 * @brief  PI控制器更新计算 (已修正积分抗饱和逻辑)
 * @param  pi: PI控制器结构体指针
 * @param  setpoint: 设定值
 * @param  feedback: 反馈值
 * @retval PI控制器输出值
 */
float PI_Update(PI_Controller *pi, float setpoint, float feedback)
{
  // 1. 计算误差
  float error = setpoint - feedback;

  // 2. 计算PI输出
  // (注意：这里我们先计算总输出，不在这里限幅 integral)
  float output = pi->Kp * error + pi->Ki * pi->integral;

  // 3. 总输出限幅
  // (先计算，再限幅)
  float output_clamped = output;
  if (output_clamped > pi->output_max)
  {
    output_clamped = pi->output_max;
  }
  else if (output_clamped < pi->output_min)
  {
    output_clamped = pi->output_min;
  }

  //
  // 解释：
  // (error > 0 && output < pi->output_max) -> 误差为正，且输出没到顶，允许正向累积
  // (error < 0 && output > pi->output_min) -> 误差为负，且输出没到底，允许反向累积
  //
  // 如果 output >= pi->output_max 且 error > 0，积分将停止累加，防止"饱和"
  //
  if ((error > 0.0f && output < pi->output_max) || (error < 0.0f && output > pi->output_min))
  {
    pi->integral += error * 0.0005f;
  }

  // 5. 返回被限幅后的输出
  return output_clamped;
}

/**

 * @brief  读取ADC电压值 (从缓存读取,非阻塞)
 * @note   读取ADC中断已更新的转换结果
 * @retval 实际电压值(V)
 */
float ADC_ReadVoltage_HAL(void)
{
  // 直接读取ADC中断更新的值,无需等待
  float voltage = (float)adc_value * 3.3f / 4096.0f;
  return voltage * 5.965f; // 乘以分压比
}

/**
 * @brief  控制循环
 * @note   读取电压,PI计算,更新PWM占空比,启动下次ADC转换
 * @retval None
 */
void Control_Loop_HAL(void)
{
  // 1. 读取上次ADC转换的电压值 
  voltage_feedback = ADC_ReadVoltage_HAL();

  // 2. 归一化处理 (0.0 ~ 1.0)
  float feedback_norm = voltage_feedback / MAX_VOLTAGE;
  float setpoint_norm = voltage_setpoint / MAX_VOLTAGE;

  // 3. PI调节 (输入输出都是 0.0~1.0)
  float control_norm = PI_Update(&voltage_pi, setpoint_norm, feedback_norm);

  // 4. 映射回PWM占空比
  pwm_duty = (uint16_t)(control_norm * PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_duty);

  // 5. 启动下次ADC转换 (中断方式,非阻塞)
  HAL_ADC_Start_IT(&hadc1);
}

/**
 * @brief  定时器周期溢出回调函数
 * @note   当TIM2计数器溢出时,此函数会被调用 (2kHz)
 *         执行闭环控制循环
 * @param  htim: 定时器句柄指针
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // 检查是否是TIM2的中断 (控制循环)
  if (htim->Instance == TIM2)
  {
    // 执行控制循环 (只读取、计算、输出,不阻塞)
    Control_Loop_HAL();

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
}

/**
 * @brief  ADC转换完成回调函数
 * @note   当ADC转换完成时,此函数会被自动调用
 *         保存ADC结果,供下次控制循环使用
 * @param  hadc: ADC句柄指针
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    // 读取ADC转换结果并保存
    adc_value = HAL_ADC_GetValue(&hadc1);
    adc_ready = 1; // 标记数据已就绪

    // 计算有效值 (RMS)
    adc_sq_sum += (uint64_t)adc_value * adc_value;
    adc_sample_cnt++;
    if (adc_sample_cnt >= RMS_SAMPLE_COUNT)
    {
      float mean_sq = (float)adc_sq_sum / adc_sample_cnt;
      float rms_adc = sqrtf(mean_sq);
      // 转换为电压值: (RMS_ADC / 4096) * 3.3 * 5.965
      adc_rms_voltage = (rms_adc * 3.3f / 4096.0f) * 5.965f;
      
      adc_sq_sum = 0;
      adc_sample_cnt = 0;
    }
  }
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
