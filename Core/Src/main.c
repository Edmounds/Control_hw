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
#include "dma.h"
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

// 硬件参数定义 (NTC)
#define R_UP          3000.0f   // 上拉电阻 R72 = 3K
#define R_PARALLEL    5000.0f  // 并联电阻 R73 = 5K
#define V_CC          3.3f     // 供电电压 3.3V
#define B_VALUE       3950.0f  // NTC的B值 
#define R_NTC_25      20000.0f // 
#define T25           298.15f  // 25度对应的开尔文温度

// 1. 采样电阻 (R66)
#define CURRENT_SENSE_RES  0.005f  // 5毫欧

// 2. 运放放大倍数 (51k / 1k)
#define CURRENT_AMP_GAIN   51.0f   // 放大51倍

// 定义两个不同的窗口大小
#define FILTER_SIZE_FAST 10   // 给电流用 (5ms响应)
#define FILTER_SIZE_SLOW 80   // 给温度/电池用 (32ms响应，能盖过50Hz周期)
#define MAX_FILTER_SIZE  128   // 最大的那个，用于定义数组

typedef struct {
  uint16_t buffer[MAX_FILTER_SIZE]; // 数组开到最大
  uint8_t index;
  uint32_t sum;
  uint8_t count;
  uint8_t size; // 新增：每个滤波器自己的大小
} Filter_t;

typedef struct {
  float buffer[MAX_FILTER_SIZE];
  uint8_t index;
  float sum;
  uint8_t count;
  uint8_t size; // 新增
} Filter_Float_t;
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

// RMS Calculation Buffers (Main Loop)
volatile uint64_t adc_sq_sum_buf = 0;
volatile float vac_sq_sum_buf = 0.0f;
volatile uint8_t rms_ready_flag = 0; // Bit 0: ADC1, Bit 1: ADC2

// ADC DMA Buffer
volatile uint16_t adc_dma_buffer[4];
// buffer[0]: Feedback (PA2)
// buffer[1]: Temp (PA5)
// buffer[2]: Bat (PA6)
// buffer[3]: Current (PA7)

volatile uint16_t adc_vac_raw = 0;     // ADC2 PA4
volatile uint16_t adc_temp_raw = 0;    // ADC1 PA5
volatile uint16_t adc_bat_raw = 0;     // ADC1 PA6
volatile uint16_t adc_current_raw = 0; // ADC1 PA7

// Filtered Values
volatile uint16_t adc_temp_flt = 0;
volatile uint16_t adc_bat_flt = 0;
volatile uint16_t adc_current_flt = 0;
volatile float vac_rms_flt = 0.0f;

// Filter Structures
Filter_t flt_temp = {0};
Filter_t flt_bat = {0};
Filter_t flt_current = {0};
Filter_Float_t flt_vac = {0};

// VAC RMS Calculation Variables
volatile float vac_rms_voltage = 0.0f;
float vac_sq_sum = 0.0f;
uint16_t vac_sample_cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float Get_Temperature(uint16_t adc_val);
uint16_t Apply_Filter(Filter_t *f, uint16_t val);
float Apply_Filter_Float(Filter_Float_t *f, float val);
void Control_Loop_HAL(void); // Add prototype
void Filter_Init(Filter_t *f, uint8_t size);
void Filter_Float_Init(Filter_Float_t *f, uint8_t size);
/* USER CODE END PFP */

void Filter_Init(Filter_t *f, uint8_t size) {
    f->size = (size > MAX_FILTER_SIZE) ? MAX_FILTER_SIZE : size;
    f->index = 0;
    f->sum = 0;
    f->count = 0;
    for(int i=0; i<MAX_FILTER_SIZE; i++) f->buffer[i] = 0;
}

void Filter_Float_Init(Filter_Float_t *f, uint8_t size) {
    f->size = (size > MAX_FILTER_SIZE) ? MAX_FILTER_SIZE : size;
    f->index = 0;
    f->sum = 0.0f;
    f->count = 0;
    for(int i=0; i<MAX_FILTER_SIZE; i++) f->buffer[i] = 0.0f;
}/* Private user code ---------------------------------------------------------*/
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
  MX_DMA_Init();
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

  // 启动风扇PWM (TIM2_CH1)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  // 校准ADC (提高精度)
  HAL_ADCEx_Calibration_Start(&hadc1);

  // 启动 ADC1 DMA (循环模式, 自动搬运4个通道数据)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, 4);

  // --- 初始化滤波器 ---
  // 温度和电池：用慢速 (64个点 = 32ms窗口)，为了稳！
  Filter_Init(&flt_temp, FILTER_SIZE_SLOW);
  Filter_Init(&flt_bat, FILTER_SIZE_SLOW);
  
  // 电流
  Filter_Init(&flt_current, FILTER_SIZE_SLOW);
  
  // 市电RMS：它本身更新就慢，稍微平滑一下即可 (比如4)
  Filter_Float_Init(&flt_vac, 2); 

  // 启动控制循环定时器 (TIM2)
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // --- RMS 计算处理 (移出中断) ---
    if (rms_ready_flag & 0x01) // ADC1 Feedback
    {
      float mean_sq = (float)adc_sq_sum_buf / RMS_SAMPLE_COUNT;
      float rms_adc = sqrtf(mean_sq);
      adc_rms_voltage = (rms_adc * 3.3f / 4096.0f) * 5.965f;
      rms_ready_flag &= ~0x01;
    }

    if (rms_ready_flag & 0x02) // ADC2 VAC
    {
      float mean_sq = vac_sq_sum_buf / RMS_SAMPLE_COUNT;
      vac_rms_voltage = sqrtf(mean_sq);
      vac_rms_flt = Apply_Filter_Float(&flt_vac, vac_rms_voltage);
      rms_ready_flag &= ~0x02;
    }
    // -----------------------------

    char buf[20];

    // 1. 稳压反馈
    (void)snprintf(buf, sizeof(buf), "V:%-5.2fV", adc_rms_voltage);
    OLED_ShowString(0, 0, buf, 12);

    // 2. 市电电压
    // float vac_val = (float)adc_vac_raw * 3.3f / 4096.0f; 
    (void)snprintf(buf, sizeof(buf), "AC:%-4.2fV", vac_rms_flt*404.0f);
    OLED_ShowString(0, 2, buf, 12);

    // 3. 温度 & 电池 
    float temp_val = Get_Temperature(adc_temp_flt) - 85.0f;
    float bat_val = ((float)adc_bat_flt * 3.3f / 4096.0f) * 12.0f;
    (void)snprintf(buf, sizeof(buf), "T:%-3.1f B:%-3.1f", temp_val, bat_val);
    OLED_ShowString(0, 4, buf, 12);

    // 4. 母线电流 
    float v_pin = (float)adc_current_flt * 3.3f / 4096.0f;
    float current_actual = v_pin / (CURRENT_SENSE_RES * CURRENT_AMP_GAIN);

    if (current_actual < 0.1f) current_actual = 0.0f; // 强制归零小电流噪音

    (void)snprintf(buf, sizeof(buf), "I:%-4.2fA", current_actual/11.0f);
    OLED_ShowString(0, 6, buf, 12);

    HAL_Delay(50); // 刷新率 20Hz

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
 * @brief  控制循环
 * @note   读取电压,PI计算,更新PWM占空比
 * @retval None
 */
void Control_Loop_HAL(void)
{
  // 1. 直接从 DMA 数组取值 (Feedback = buffer[0])
  uint16_t raw_feedback = adc_dma_buffer[0];
  
  // 转换电压
  voltage_feedback = ((float)raw_feedback * 3.3f / 4096.0f) * 5.965f;

  // --- RMS 计算 (Feedback) ---
  // 移入控制循环，保持与之前类似的统计逻辑
  adc_sq_sum += (uint64_t)raw_feedback * raw_feedback;
  adc_sample_cnt++;
  if (adc_sample_cnt >= RMS_SAMPLE_COUNT)
  {
    adc_sq_sum_buf = adc_sq_sum;
    rms_ready_flag |= 0x01; // 标记ADC1数据就绪
    
    adc_sq_sum = 0;
    adc_sample_cnt = 0;
  }
  // ---------------------------

  // --- 更新其他传感器滤波值 ---
  // Temp (buffer[1]), Bat (buffer[2]), Current (buffer[3])
  adc_temp_flt = Apply_Filter(&flt_temp, adc_dma_buffer[1]);
  adc_bat_flt = Apply_Filter(&flt_bat, adc_dma_buffer[2]);
  adc_current_flt = Apply_Filter(&flt_current, adc_dma_buffer[3]);
  // ---------------------------

  // 2. 归一化处理 (0.0 ~ 1.0)
  float feedback_norm = voltage_feedback / MAX_VOLTAGE;
  float setpoint_norm = voltage_setpoint / MAX_VOLTAGE;

  // 3. PI调节 (输入输出都是 0.0~1.0)
  float control_norm = PI_Update(&voltage_pi, setpoint_norm, feedback_norm);

  // 4. 映射回PWM占空比
  pwm_duty = (uint16_t)(control_norm * PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_duty);

  // 5. 启动 ADC2 (VAC) 转换 (ADC1 由 DMA 自动处理，无需手动启动)
  HAL_ADC_Start_IT(&hadc2); 
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
  if (hadc->Instance == ADC2)
  {
    adc_vac_raw = HAL_ADC_GetValue(&hadc2);

    // 市电电压有效值计算 (RMS)
    // 1. 转换为电压值 (0-3.3V)
    float v_in = (float)adc_vac_raw * 3.3f / 4096.0f;
    // 2. 减去直流偏置 1.5V
    float v_ac = v_in - 1.5f;
    // 3. 累加平方和
    vac_sq_sum += v_ac * v_ac;
    vac_sample_cnt++;

    if (vac_sample_cnt >= RMS_SAMPLE_COUNT)
    {
      // 4. 计算均方根
      vac_sq_sum_buf = vac_sq_sum;
      rms_ready_flag |= 0x02; // 标记ADC2数据就绪
      // 清零重新累计
      vac_sq_sum = 0.0f;
      vac_sample_cnt = 0;
    }
  }
}
/**
 * @brief  滑动平均滤波 (uint16_t)
 */
uint16_t Apply_Filter(Filter_t *f, uint16_t val)
{
  f->sum -= f->buffer[f->index];
  f->buffer[f->index] = val;
  f->sum += val;
  
  f->index++;
  if (f->index >= f->size)
  {
    f->index = 0;
  }

  if (f->count < f->size)
  {
    f->count++;
  }

  return (uint16_t)(f->sum / f->count);
}

/**
 * @brief  滑动平均滤波 (float)
 */
float Apply_Filter_Float(Filter_Float_t *f, float val)
{
  f->sum -= f->buffer[f->index];
  f->buffer[f->index] = val;
  f->sum += val;
  
  f->index++;
  if (f->index >= f->size)
  {
    f->index = 0;
  }

  if (f->count < f->size)
  {
    f->count++;
  }

  return f->sum / f->count;
}

// 20K B3950 Steinhart-Hart 系数
#define SH_A 0.001016186f
#define SH_B 0.000224868f
#define SH_C 0.0000001156f 

float Get_Temperature(uint16_t adc_val)
{
    // 1. 算出引脚电压
    float v_pin = (float)adc_val * 3.3f / 4096.0f;
    
    // 保护：防止除以0或电压异常
    if(v_pin >= V_CC || v_pin <= 0.1f) return 0.0f;

    // 2. 反推 NTC 电阻值 (保持你原有的电路逻辑)
    // R_down = (V_pin * R_UP) / (V_cc - V_pin)
    float r_down = (v_pin * R_UP) / (V_CC - v_pin);

    // 剥离并联电阻 R_PARALLEL (如果你板子上还有 R73 5K 的话)
    // 如果没有 R73，直接用 r_ntc = r_down
    float r_ntc = 1.0f / ( (1.0f / r_down) - (1.0f / R_PARALLEL) );

    // 再次检查负值（防止并联电阻计算溢出）
    if (r_ntc <= 0.0f) return 0.0f; 

    // 3. 使用拟合好的 Steinhart-Hart 公式计算温度
    float log_r = logf(r_ntc); // 取自然对数
    float log_r3 = log_r * log_r * log_r; // 对数的立方

    // Steinhart-Hart 公式: 1/T = A + B*ln(R) + C*(ln(R))^3
    float temp_kelvin = 1.0f / (SH_A + SH_B * log_r + SH_C * log_r3);
    
    // 转摄氏度
    float temp_celsius = temp_kelvin - 273.15f;

    return temp_celsius;
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
