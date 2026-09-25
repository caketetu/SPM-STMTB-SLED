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
#include "dma.h"
#include "dmamux.h"
#include "fdcan.h"
#include "spi.h"
#include "stm32c0xx_hal_tim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "SPM_Modbus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define WS_ZERO   18
#define WS_ONE    38
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_MAX     32
#define RESET_SLOTS 120
#define WS_BITS     24
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//デバイス定義
int16_t model_no = 51;
int16_t version = 1;
int16_t sub_version = 0;

//シリアルデータ
uint8_t recv_data;
uint8_t rx_buf[256];
uint8_t tx_buf[256];
int rx_len=0;

//LED制御用変数
uint16_t pwmData[LED_MAX * WS_BITS + RESET_SLOTS];
volatile int pwmDMA_busy[4] = {0,0,0,0}; // DMA送信中フラグ
//パターン点灯用変数
volatile int16_t ten_ms_timer = 0; // 10msごとのタイマー
int16_t pattern_step = 0; //パターン点灯のステップ数
uint8_t pattern_red = 0; // 徐々に赤の値
uint8_t pattern_green = 0; // 徐々に緑の値
uint8_t pattern_blue = 0; // 徐々に青の値
int8_t flame_variation[LED_MAX] = {0};
uint32_t flame_random_state = 0x13579BDFU;

//PWMチャンネル指定
// TIM1 ch1, TIM1 ch2, TIM2 ch4, TIM3 ch4
#define PWM_CH_COUNT 4
//TIM_HandleTypeDef *pwm_timers[4] = {&htim1, &htim1, &htim2, &htim3};
//uint32_t pwm_channels[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_4, TIM_CHANNEL_4};
TIM_HandleTypeDef *pwm_timers[4] = {&htim1, &htim1, &htim1, &htim1};
uint32_t pwm_channels[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

//アプリケーション変数
int16_t dev_id=51; //デバイスID
//LEDのRGBデータ 1チャンネルで最大32個までのLEDを制御可能
//一つのLEDは8bit×24bitのデータが必要だが、レジスタは16bitなので、2つのレジスタに分けて格納する
// ledData[0] = LED1_R, LED1_G
// ledData[1] = LED1_B, LED2_R
// ledData[2] = LED2_G, LED2_B
// ...
int16_t ledData[48]; // LEDのRGBデータ
int16_t mode = 0; //動作モード　0:レジスタより点灯, 1:全LED同時点灯, 2~5:パターン点灯モード
int16_t brightness_lim = 16; //明るさの上限
int16_t led_num = 16; //LEDの数
int16_t safety_mode = 0; //セーフティモード… 0:無効, 1:有効　有効の時は強制的に赤に点灯
int16_t safety_flg = 0; //セーフティモード時の赤の明るさ
int16_t safety_r = 16; //セーフティモード時の赤の明るさ
int16_t enable_input = 0; //イネーブル信号入力

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef WS2812_Send(int pwm_no);
static HAL_StatusTypeDef WS2812_StartSend(int pwm_no);
uint8_t limit_max(uint8_t value, uint8_t max);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void WS2812_SetLED(uint8_t led,uint8_t r,uint8_t g,uint8_t b)
{
    uint32_t color;

    r = limit_max(r, brightness_lim);
    g = limit_max(g, brightness_lim);
    b = limit_max(b, brightness_lim);

    color =
        ((uint32_t)g << 16) |
        ((uint32_t)r << 8 ) |
         b;

    int index = led * 24;

    for(int i=0;i<24;i++)
    {
        if(color & (1<<(23-i)))
            pwmData[index+i]=WS_ONE;
        else
            pwmData[index+i]=WS_ZERO;
    }
}

void WS2812_Reset(void)
{
    for(int i=led_num*24;
        i<led_num*24+RESET_SLOTS;
        i++)
    {
        pwmData[i]=0;
    }
}

  static HAL_StatusTypeDef WS2812_Send(int pwm_no)
  {
    HAL_StatusTypeDef status;
    uint32_t start_tick;

    pwmDMA_busy[pwm_no] = 1;
    status = HAL_TIM_PWM_Start_DMA(pwm_timers[pwm_no],
                     pwm_channels[pwm_no],
                     (uint32_t *)pwmData,
                     led_num * WS_BITS + RESET_SLOTS);
    if (status != HAL_OK)
    {
      pwmDMA_busy[pwm_no] = 0;
      return status;
    }

    start_tick = HAL_GetTick();
    while (pwmDMA_busy[pwm_no] != 0)
    {
      if ((HAL_GetTick() - start_tick) > 100U)
      {
        HAL_TIM_PWM_Stop_DMA(pwm_timers[pwm_no], pwm_channels[pwm_no]);
        pwmDMA_busy[pwm_no] = 0;
        return HAL_TIMEOUT;
      }
    }

    return HAL_TIM_PWM_Stop_DMA(pwm_timers[pwm_no], pwm_channels[pwm_no]);
  }

  /* DMA送信を開始するだけで完了を待たない（TIM14の10ms処理から呼び出す用） */
  static HAL_StatusTypeDef WS2812_StartSend(int pwm_no)
  {
    HAL_StatusTypeDef status;

    pwmDMA_busy[pwm_no] = 1;
    status = HAL_TIM_PWM_Start_DMA(pwm_timers[pwm_no],
                     pwm_channels[pwm_no],
                     (uint32_t *)pwmData,
                     led_num * WS_BITS + RESET_SLOTS);
    if (status != HAL_OK)
    {
      pwmDMA_busy[pwm_no] = 0;
    }

    return status;
  }


//LED明るさ制限
uint8_t limit_max(uint8_t value, uint8_t max)
{
    if(value > max)
        return max;
    return value;
}

static uint8_t flame_random(void)
{
    flame_random_state ^= flame_random_state << 13;
    flame_random_state ^= flame_random_state >> 17;
    flame_random_state ^= flame_random_state << 5;
    return (uint8_t)(flame_random_state & 0xFFU);
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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_DMAMUX_Init();
  /* USER CODE BEGIN 2 */

  //各レジスタ初期化
  SPM_ModbusInit();
  //レジスタ登録
  p_input_regs[0] = &model_no;   //モデルナンバー
  p_input_regs[1] = &version; //バージョン
  p_input_regs[2] = &sub_version; //サブバージョン
  for(int i=3;i<51;i++)
  {
      p_input_regs[i] = &ledData[i-3];
  }
  p_input_regs[51] = &safety_flg;
  p_input_regs[52] = &enable_input;

  p_holding_regs[0] = &dev_id;
  for(int i=1;i<49;i++)
  {
      p_holding_regs[i] = &ledData[i-1];
  }
  p_holding_regs[49] = &mode;
  p_holding_regs[50] = &brightness_lim;
  p_holding_regs[51] = &led_num;
  p_holding_regs[52] = &safety_mode;
  p_holding_regs[53] = &safety_flg;
  p_holding_regs[54] = &safety_r;

  //Cyclic Function登録
  // cycfunc0.rx_len = 2;
  // cycfunc0.rx_adr[0] = &io_in;
  // cycfunc0.rx_adr[1] = &io_out;
  // cycfunc0.tx_len = 1;
  // cycfunc0.tx_adr[0] = &io_out;

  ParamLoad();
  SPM_ModbusSetAddress(dev_id);
  if(led_num > 64) led_num = 64;

  HAL_UART_Receive_IT(&huart2, &recv_data, 1);

  HAL_TIM_Base_Start_IT(&htim14); // Start TIM14 for periodic interrupts
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // LED ON

  //ledData初期化
  for(int i=0;i<(int)(sizeof(ledData) / sizeof(ledData[0]));i++)
  {
      ledData[i] = 0;
  }

  /* NeoPixel data send first so CAN init errors do not mask LED bring-up */
  //すべてのLEDを白にセット
  for(int i=0;i<led_num;i++)
  {
      WS2812_SetLED(i,16,16,16); //白
  }

  //送信
  for(int i=0;i<PWM_CH_COUNT;i++){
    if (WS2812_StartSend(i) != HAL_OK)
    {
      Error_Handler();
    }
  }

  HAL_Delay(500); // 1秒待機

  //再度消灯
  for(int i=0;i<led_num;i++)
  {
      WS2812_SetLED(i,0,0,0); //消灯
  }
  //送信
  for(int i=0;i<PWM_CH_COUNT;i++){
    if (WS2812_StartSend(i) != HAL_OK)
    {
      Error_Handler();
    }
  }

  HAL_Delay(500); // 1秒待機

  //ledデータ初期化
  for(int i=0;i<led_num;i++)
  {
      WS2812_SetLED(i,0,0,0); //消灯
  }
  /* 送信 */
  for(int i=0;i<PWM_CH_COUNT;i++){
    if (WS2812_StartSend(i) != HAL_OK)
    {
      Error_Handler();
    }
  }
  

  //FDCAN1 filter configuration
  // FDCAN_FilterTypeDef sFilterConfig;
  // sFilterConfig.IdType = FDCAN_STANDARD_ID;
  // sFilterConfig.FilterIndex = 0;
  // sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  // sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  // sFilterConfig.FilterID1 = 0x123; // Standard ID is configured as-is (no manual bit shift)
  // sFilterConfig.FilterID2 = 0x7FF; // Filter mask (accept all IDs)
  // if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  // {
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Filter Configuration Error\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  //   // Filter configuration Error
  //   Error_Handler();
  // }else{
    
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Filter Configuration Success\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  // }

  // // Start FDCAN module
  // if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  // {
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Start Error\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  //   // Start Error
  //   Error_Handler();
  // }else{
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Start Success\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  // }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // //FDCAN送信
    // FDCAN_TxHeaderTypeDef TxHeader = {0};
    // uint8_t CanTxData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, counter++}; // 送信データ
    // TxHeader.Identifier = 0x143; // 送信するID
    // TxHeader.IdType = FDCAN_STANDARD_ID;
    // TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    // TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    // TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    // TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    // TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    // TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    // TxHeader.MessageMarker = 0;
    // if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CanTxData) != HAL_OK)
    // {
    //   // Transmission request Error
    //   TxDataLength = sprintf((char*)TxData, "FDCAN Transmission Error\r\n");
    //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
    //   Error_Handler();
    // }else{
    //   TxDataLength = sprintf((char*)TxData, "FDCAN Transmission Success\r\n");
    //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
    // }

    //シリアル通信処理
		if(rx_len>0){
		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
			HAL_Delay(10);
			int16_t tx_len = SPM_ModbusTask(rx_buf, rx_len, tx_buf);
			HAL_UART_Transmit(&huart2, tx_buf, tx_len, 100);
			rx_len=0;
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		}

    //イネーブル信号確認
    enable_input = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5); // イネーブル信号の状態を読み取る 
    if(enable_input==1)
    {
        // イネーブル信号が有効な場合の処理をここに記述
        safety_flg = 1; // 安全モードを有効にする
    }

    if(safety_mode && safety_flg)
    {
        // 安全モード時の処理をここに記述
        // 全LED赤に点灯
        for(int i=0;i<led_num;i++)
        { 
          WS2812_SetLED(i,safety_r,0,0); // 赤
        }
    }else{
        // 安全モードでない場合の処理をここに記述
        switch(mode){
          case 0:
              // モード0…レジスタから点灯モード
              uint8_t r[32];
              uint8_t g[32];
              uint8_t b[32];
              int r_index = 0;
              int g_index = 0;
              int b_index = 0;
              for(int i=0;i<48;i++){
                  if(i%3==0){
                      r[r_index] = (uint8_t)(ledData[i] >> 8);
                      g[g_index] = (uint8_t)(ledData[i] & 0xFF);
                      r_index++;
                      g_index++;
                  }else if(i%3==1){
                      b[b_index] = (uint8_t)(ledData[i] >> 8);
                      r[r_index] = (uint8_t)(ledData[i] & 0xFF);
                      b_index++;
                      r_index++;
                  }else if(i%3==2){
                      g[g_index] = (uint8_t)(ledData[i] >> 8);
                      b[b_index] = (uint8_t)(ledData[i] & 0xFF);
                      g_index++;
                      b_index++;
                  }
              }
              //LEDの色をNeoPixelに送信
              for(int i=0;i<led_num;i++)
              {
                  WS2812_SetLED(i,r[i],g[i],b[i]);
              }
              break;
          case 1:
              // モード1…固定色点灯モード
              uint8_t fixed_r = ledData[0] >> 8;
              uint8_t fixed_g = ledData[0] & 0xFF;
              uint8_t fixed_b = ledData[1] >> 8;
              for(int i=0;i<led_num;i++)
              {
                  WS2812_SetLED(i,fixed_r,fixed_g,fixed_b);
              }
              break;
          case 2:
              // モード2…パターン点灯モード1
              // 全LEDを同じ色で点灯
              //赤→黄色→緑→シアン→青→マゼンタ→赤の順で点灯させるパターン
              //各色の切り替わりの時は徐々に明るさを変えるようにする
              switch(pattern_step){
                  case 0:
                      // 徐々にマゼンタから赤へ R=MAX G=0 B=DOWN
                      pattern_red = brightness_lim;
                      pattern_green = 0;
                      pattern_blue = (uint8_t)(brightness_lim * (100 - ten_ms_timer) / 100.0);
                      break;
                  case 1:
                      // 徐々に赤から黄色へ R=MAX G=UP B=0
                      pattern_red = brightness_lim;
                      pattern_green = (uint8_t)(brightness_lim * ten_ms_timer / 100.0);
                      pattern_blue = 0;
                      break;
                  case 2:
                      // 徐々に黄色から緑へ R=DOWN G=MAX B=0
                      pattern_red = (uint8_t)(brightness_lim * (100 - ten_ms_timer) / 100.0);
                      pattern_green = brightness_lim;
                      pattern_blue = 0;
                      break;
                  case 3:
                      // 徐々に緑からシアンへ R=0 G=MAX B=UP
                      pattern_red = 0;
                      pattern_green = brightness_lim;
                      pattern_blue = (uint8_t)(brightness_lim * ten_ms_timer / 100.0);
                      break;
                  case 4:
                      // 徐々にシアンから青へ R=0 G=DOWN B=MAX
                      pattern_red = 0;
                      pattern_green = (uint8_t)(brightness_lim * (100 - ten_ms_timer) / 100.0);
                      pattern_blue = brightness_lim;
                      break;
                  case 5:
                      // 徐々に青からマゼンタへ R=UP G=0 B=MAX
                      pattern_red = (uint8_t)(brightness_lim * ten_ms_timer / 100.0);
                      pattern_green = 0;
                      pattern_blue = brightness_lim;
                      break;
                  default:
                      break;
              }
              for(int i=0;i<led_num;i++)
              { 
                WS2812_SetLED(i,pattern_red,pattern_green,pattern_blue); 
              }
              if(ten_ms_timer >= 100){
                  ten_ms_timer = 0;
                  pattern_step++;
                  if(pattern_step > 5) pattern_step = 0;
              }
              break;
          case 3:
              // モード3…パターン点灯モード2
              // ひとつだけ点灯する
              // 100msごとに点灯するLEDを切り替える
              //位置によって点灯するLEDの色を変える
              for(int i=0;i<led_num;i++)
              {
                  if(i == pattern_step) {
                      //赤は最初初期位置が最大、徐々に減少させる
                      uint8_t color_red = 0;
                      //緑は真ん中で最大になるように設定
                      uint8_t color_green = 0;
                      //青は最初初期位置が0、徐々に増加させる
                      uint8_t color_blue = 0;
                      if (led_num > 1)
                      {
                        float center = (led_num - 1) / 2.0f;
                        if (i <= center)
                        {
                          float position = i / center;
                          color_red = (uint8_t)(brightness_lim * (1.0f - position));
                          color_green = (uint8_t)(brightness_lim * position);
                        }
                        else
                        {
                          float position = (i - center) / center;
                          color_green = (uint8_t)(brightness_lim * (1.0f - position));
                          color_blue = (uint8_t)(brightness_lim * position);
                        }
                      }
                      WS2812_SetLED(i,color_red,color_green,color_blue); // シアン
                  } else {
                      WS2812_SetLED(i,0,0,0); // 消灯
                  }
              }
              if(ten_ms_timer >= 10){
                  ten_ms_timer = 0;
                  pattern_step++;
                  if(pattern_step >= led_num) pattern_step = 0;
              }
              break;
          case 4:
              // モード4…パターン点灯モード3
              // 炎をイメージした点灯パターン
              // 暖色系で炎のような色合いを表現する
              // ledの位置によって色を変えることで炎の揺らぎを表現する
              // 位置が小さいほど温度が高く、上に行くほど温度が低くなるように色を設定する
              //100msごとにランダムにわずかに色を変化させ揺らぎを表現する
                if (ten_ms_timer >= 10)
                {
                  ten_ms_timer = 0;
                  for (int i = 0; i < LED_MAX; i++)
                  {
                    flame_variation[i] = (int8_t)(flame_random() % 13U) - 6;
                  }
                }

                int flame_leds = led_num;
                if (flame_leds > LED_MAX)
                {
                  flame_leds = LED_MAX;
                }
                for (int i = 0; i < flame_leds; i++)
                {
                  float position = flame_leds > 1
                    ? i / (float)(flame_leds - 1)
                    : 0.0f;
                  int16_t red = brightness_lim + flame_variation[i];
                  int16_t green = (int16_t)(brightness_lim *
                    (0.85f - 0.70f * position)) + flame_variation[i];

                  if (red < 0) red = 0;
                  if (green < 0) green = 0;

                  WS2812_SetLED(i, (uint8_t)red, (uint8_t)green, 0);
                }
              break;
          case 5:
              // モード5…パターン点灯モード4
              // 青い炎をイメージした点灯パターン
              // 寒色系で炎のような色合いを表現する
              // ledの位置によって色を変えることで炎の揺らぎを表現する
              // 位置が小さいほど温度が高く、上に行くほど温度が低くなるように色を設定する
              //100msごとにランダムにわずかに色を変化させ揺らぎを表現する
              {
                if (ten_ms_timer >= 10)
                {
                  ten_ms_timer = 0;
                  for (int i = 0; i < LED_MAX; i++)
                  {
                    flame_variation[i] = (int8_t)(flame_random() % 13U) - 6;
                  }
                }

                int flame_leds = led_num;
                if (flame_leds > LED_MAX)
                {
                  flame_leds = LED_MAX;
                }
                for (int i = 0; i < flame_leds; i++)
                {
                  float position = flame_leds > 1
                    ? i / (float)(flame_leds - 1)
                    : 0.0f;
                  int16_t red = (int16_t)(brightness_lim *
                    (0.15f - 0.15f * position)) + flame_variation[i];
                  int16_t green = (int16_t)(brightness_lim *
                    (0.80f - 0.60f * position)) + flame_variation[i];
                  int16_t blue = brightness_lim + flame_variation[i];

                  if (red < 0) red = 0;
                  if (green < 0) green = 0;
                  if (blue < 0) blue = 0;

                  WS2812_SetLED(i, (uint8_t)red, (uint8_t)green,
                    (uint8_t)blue);
                }
              break;
              }
          default:
              // その他のモードの処理をここに記述
              
              break;
        }
    }
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */
/* pwm_channels[] (TIM_CHANNEL_x) に対応する HAL_TIM_ActiveChannel 一覧 */
static const HAL_TIM_ActiveChannel pwm_active_channels[PWM_CH_COUNT] = {
    HAL_TIM_ACTIVE_CHANNEL_1, /* TIM1 CH1 */
    HAL_TIM_ACTIVE_CHANNEL_2, /* TIM1 CH2 */
    HAL_TIM_ACTIVE_CHANNEL_3, /* TIM1 CH3 */
    HAL_TIM_ACTIVE_CHANNEL_4 /* TIM1 CH4 */
};

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    for (int i = 0; i < PWM_CH_COUNT; i++)
    {
        if (pwm_timers[i]->Instance == htim->Instance &&
            htim->Channel == pwm_active_channels[i])
        {
            __HAL_TIM_SET_COMPARE(pwm_timers[i], pwm_channels[i], 0);
            pwmDMA_busy[i] = 0; // DMA送信完了フラグをクリア
            break;
        }
    }
}

volatile int timer_1s = 0; // タイマーカウンタ
volatile int timer_10ms = 0; // タイマーカウンタ
volatile int led_state[4] = {0,0,0,0}; // タイマーカウンタ
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM14)
    {
        timer_1s++;
        if(timer_1s>=1000)
        {
            timer_1s=0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle LED to indicate 1 second elapsed
        }
        timer_10ms++;
        if(timer_10ms>=10)
        {
            timer_10ms=0;
            ten_ms_timer++;
            switch(led_state[0])
            {
                case 0:
                    led_state[0] = 1;
                    for (int i = 0; i < PWM_CH_COUNT; i++)
                    {
                        WS2812_StartSend(i);
                    }
                    break;
                case 1:
                    led_state[0] = 2;
                    break;
                case 2:
                    led_state[0] = 0;
                    for (int i = 0; i < PWM_CH_COUNT; i++)
                    {
                        HAL_TIM_PWM_Stop(pwm_timers[i], pwm_channels[i]);
                    }
                    break;
                default:
                    led_state[0] = 0;
                    break;
            }
            // for(int i=0;i<PWM_CH_COUNT;i++){
            //   switch(led_state[i])
            //   {
            //       case 0: // 送信開始
            //           if (WS2812_StartSend(i) == HAL_OK)
            //           {
            //               led_state[i] = 1;
            //           }
            //           break;
            //       case 1: // 送信完了待ち
            //           if (pwmDMA_busy[i] == 0)
            //           {
            //               /* HAL_TIM_PWM_Stop()はTIM1のようなアドバンストタイマーでは
            //                  MOE(メイン出力)とカウンタ(CEN)をタイマー全体で無効化してしまう。
            //                  同じタイマーを共有する他チャンネルがまだ送信中の間に呼ぶと、
            //                  そのチャンネルの出力/DMA転送まで巻き添えで止まってしまうため、
            //                  同一タイマーを使う全チャンネルの送信が完了してから呼び出す。 */
            //               int timer_done = 1;
            //               for (int j = 0; j < PWM_CH_COUNT; j++)
            //               {
            //                   if (pwm_timers[j]->Instance == pwm_timers[i]->Instance &&
            //                       pwmDMA_busy[j] != 0)
            //                   {
            //                       timer_done = 0;
            //                       break;
            //                   }
            //               }
            //               if (timer_done)
            //               {
            //                   for (int j = 0; j < PWM_CH_COUNT; j++)
            //                   {
            //                       if (pwm_timers[j]->Instance == pwm_timers[i]->Instance)
            //                       {
            //                           HAL_TIM_PWM_Stop(pwm_timers[j], pwm_channels[j]);
            //                       }
            //                   }
            //               }
            //               led_state[i] = 2;
            //           }
            //           break;
            //       case 2: // 次サイクルへ
            //           led_state[i] = 0;
            //           break;
            //       default:
            //           led_state[i] = 0;
            //           break;
            //   }
            // }
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == USART2)
   {
	   rx_buf[rx_len] = recv_data;
	   rx_len++;
       HAL_UART_Receive_IT(&huart2, &recv_data, 1);
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
  while(1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4); // Toggle LED to indicate error
    HAL_Delay(100); // 0.1秒待機
  }
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
