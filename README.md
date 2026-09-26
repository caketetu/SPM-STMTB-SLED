# SPM-STMTB-SLED

![MIT](https://img.shields.io/badge/license-MIT-blue.svg)
![Version](https://img.shields.io/badge/version-1.0.0-brightgreen.svg)
![STM32](https://img.shields.io/badge/MCU-STM32C092KBT6-blue.svg)

## 概要

SPM-STMTBを使用したフルカラーLED御ボード。LEDポートは4ch実装した。
SK6812フルカラーLEDにて動作を確認

## 機能

* 4chのフルカラーLED制御信号を出力
* 1chあたり最大32LEDの接続
* バッファ書き込みモード
* 全LED同時制御モード
* 三種類のサンプルモード
* RS485,CANインターフェース
* Enable信号入力によるLED制御の上書き

## ピンアサイン
| Function       | Pin  | 備考 |
| -------------- | ---- | ---- |
| TIM2_CH1       | PA0  |      |
| TIM2_CH2       | PA1  |      |
| USART2_TX      | PA2  |   SYS-RS485   |
| GPIO_Input     | PA3  |   H_LIMIT   |
| GPIO_Input     | PA4  |   L_LIMIT   |
| SPI1_SCK       | PA5  |   SYS-SPI   |
| SPI1_MISO      | PA6  |   SYS-SPI   |
| SPI1_MOSI      | PA7  |   SYS-SPI   |
| TIM1_CH1       | PA8  |      |
| USART1_TX      | PA9  |  SYS-UART    |
| USART1_RX      | PA10 |  SYS-UART    |
| FDCAN1_RX      | PA11 |  SYS-CANFD    |
| FDCAN1_TX      | PA12 |  SYS-CANFD    |
| DEBUG_SWDIO    | PA13 |  SYS-DEBUG    |
| DEBUG_SWCLK    | PA14 |  SYS-DEBUG    |
| USART2_RX      | PA15 |  SYS-RS485    |
| SPI1_NSS       | PB0  |  SYS-SPI    |
| GPIO_Output    | PB1  |      |
| GPIO_Output    | PB2  |     |
| GPIO_Output    | PB3  |      |
| GPIO_Output    | PB4  |  SYS-LED1    |
| GPIO_Input     | PB5  |  SYS-EN    |
| GPIO_Output    | PB6  |      |
| GPIO_Output    | PB7  |      |
| GPIO_Output    | PB8  |      |
| GPIO_Input     | PB9  |      |
| GPIO_Output    | PC6  |   SYS-LED0    |
| RCC_OSCIN      | PC14 |   SYS-CLOCK   |
| RCC_OSCOUT     | PC15 |   SYS-CLOCK   |
| RCC_MCO        | PF2  |   SYS-DEBUG   |

※SYS-はSPM共通インターフェース

## 開発環境

| 項目       | 内容                      |
| -------- | ----------------------- |
| MCU      | STM32C092KBT6           |
| IDE      | STM32CubeIDE for VSCode |
| CubeMX   | STM32CubeMX 6.15.0      |
| Firmware | STM32CubeC0             |
| Language | C                       |

## 実装予定機能

* [ ] 4chぶんのバッファ制御
