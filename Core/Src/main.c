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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "OLED.h"
#include "OLED_Font.h"
#include "nrf24l01p.h"
#include "ina226.h"
#include "stm32f1xx_hal_flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NRF_PAIR_ID 0u

static const uint8_t nrf_pair_addrs[][7] = {
   {0x12, 0x34, 0x56, 0x78, 0x9A}, // ID0
   {0xA1, 0xB2, 0xC3, 0xD4, 0xE5}, // ID1
   {0x57, 0x66, 0x72, 0x81, 0x99}, // ID2
   {0xA6, 0xCD, 0xEC, 0x18, 0x22}, // ID3
   {0x13, 0x57, 0x9B, 0xDF, 0x24}, // ID4
   {0x14, 0x58, 0x97, 0xD1, 0x25}, // ID5
   {0x00, 0x00, 0x00, 0x02, 0x08}, // ID6
   {0x15, 0x97, 0x9A, 0xDF, 0x74}, // ID7
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t tx_battery_voltage = 0;  // TX电池电压 (0.1V单位，75=7.5V)
static uint8_t  tx_low_voltage_alarm = 0; // TX低压报警标志
static uint32_t alarm_beep_counter = 0;   // 报警蜂鸣计数器

// NRF24L01通信监控变量
static uint32_t nrf_last_rx_time = 0;
static uint8_t  nrf_recovery_count = 0;
static uint32_t nrf_health_check_time = 0;
static uint32_t nrf_status_check_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void NRF_Force_Set_Addr(uint8_t reg, uint8_t* addr) {
  uint8_t tx_buf[6];
  tx_buf[0] = NRF24L01P_CMD_W_REGISTER | reg;
  for(int i=0; i<5; i++) tx_buf[i+1] = addr[i];
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, tx_buf, 6, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}

// 为 RX 板添加蜂鸣器驱动函数 (控制 PB3)
// 非阻塞式蜂鸣器控制变量
static uint32_t beep_end_time = 0;
static uint8_t beep_active = 0;
static uint8_t beep_sequence_step = 0;
static uint32_t beep_sequence_delay = 0;

void RX_Beep_Start(uint16_t duration_ms) {
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  beep_end_time = HAL_GetTick() + duration_ms;
  beep_active = 1;
  beep_sequence_step = 0; // 重置序列
}

// 关机提示音序列：50ms响，50ms停，50ms响
void RX_Beep_Shutdown_Sequence(void) {
  if (!beep_active) {
    beep_sequence_step = 1; // 第一步：开始第一个蜂鸣
    beep_end_time = HAL_GetTick() + 50;
    beep_active = 1;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  }
}

void RX_Beep_Update(void) {
  if (beep_active) {
    if (HAL_GetTick() >= beep_end_time) {
      if (beep_sequence_step == 1) {
        // 第一步结束，开始50ms间隔
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        beep_sequence_step = 2;
        beep_end_time = HAL_GetTick() + 50;
      } else if (beep_sequence_step == 2) {
        // 间隔结束，开始第二个蜂鸣
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
        beep_sequence_step = 3;
        beep_end_time = HAL_GetTick() + 50;
      } else {
        // 序列结束或单次蜂鸣结束
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        beep_active = 0;
        beep_sequence_step = 0;
      }
    }
  }
}

// 兼容旧代码
void RX_Beep(uint16_t duration_ms) {
  RX_Beep_Start(duration_ms);
}

// ===================== NRF24L01 通信监控函数 =====================

// NRF状态检查回调（每5秒调用一次）
void NRF_Status_Check(void) {
  if (HAL_GetTick() - nrf_status_check_time > 5000) {
    nrf_status_check_time = HAL_GetTick();
    uint8_t status = NRF_Force_Read_Reg(NRF24L01P_REG_STATUS);
    printf("[NRF] STATUS=0x%02X\r\n", status);
  }
}

// NRF健康检查和自动恢复（每30秒调用一次）
void NRF_Health_Check(void) {
  if (HAL_GetTick() - nrf_health_check_time > 30000) {
    nrf_health_check_time = HAL_GetTick();
    
    uint8_t config = NRF_Force_Read_Reg(NRF24L01P_REG_CONFIG);
    uint8_t en_rxaddr = NRF_Force_Read_Reg(NRF24L01P_REG_EN_RXADDR);
    uint8_t fifo = NRF_Force_Read_Reg(NRF24L01P_REG_FIFO_STATUS);
    
    uint8_t need_fix = 0;
    if ((config & 0x03) != 0x03) need_fix = 1;
    if ((en_rxaddr & 0x01) == 0) need_fix = 1;
    if ((fifo & 0x11) != 0x00) need_fix = 1;
    
    if (need_fix) {
      printf("[NRF] Health check: needs recovery\r\n");
      nrf24l01p_flush_rx_fifo();
      nrf24l01p_flush_tx_fifo();
      NRF_Force_Write_Reg(NRF24L01P_REG_STATUS, 0x70);
      config |= 0x03;
      NRF_Force_Write_Reg(NRF24L01P_REG_CONFIG, config);
      en_rxaddr |= 0x01;
      NRF_Force_Write_Reg(NRF24L01P_REG_EN_RXADDR, en_rxaddr);
    }
  }
}

// 统一配置应用函数：初始化和恢复时共同调用，确保地址和寄存器完整重载
void NRF_Apply_Safe_Config(void) {
  // 1. 安全解锁特性寄存器
  uint8_t feature = NRF_Force_Read_Reg(NRF24L01P_REG_FEATURE);
  if ((feature & 0x06) != 0x06) {
    NRF_Force_Write_Reg(NRF24L01P_REG_FEATURE, 0x06);
    if ((NRF_Force_Read_Reg(NRF24L01P_REG_FEATURE) & 0x06) != 0x06) {
      // 发送 ACTIVATE 命令解锁 FEATURE 寄存器（部分克隆芯片需要）
      uint8_t activate_cmd[2] = {0x50, 0x73};
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, activate_cmd, 2, 100);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      NRF_Force_Write_Reg(NRF24L01P_REG_FEATURE, 0x06);
    }
  }

  // 2. 覆盖库函数可能写错的寄存器
  NRF_Force_Write_Reg(NRF24L01P_REG_SETUP_RETR, 0x5F); // 匹配 TX
  NRF_Force_Write_Reg(NRF24L01P_REG_EN_AA, 0x01);
  NRF_Force_Write_Reg(NRF24L01P_REG_DYNPD, 0x01);
  NRF_Force_Write_Reg(NRF24L01P_REG_EN_RXADDR, 0x01);
  NRF_Force_Write_Reg(NRF24L01P_REG_RX_PW_P0, 8);      // 强制 8 字节，防乱码

  // 3. 配置配对地址（恢复时最关键的一步，防止 prx_mode 覆盖地址）
  const uint8_t *rx_addr = nrf_pair_addrs[NRF_PAIR_ID];
  NRF_Force_Set_Addr(NRF24L01P_REG_RX_ADDR_P0, (uint8_t *)rx_addr);
  NRF_Force_Set_Addr(NRF24L01P_REG_TX_ADDR, (uint8_t *)rx_addr); // ACK 应答必须发往此地址

  // 4. 屏蔽中断引脚并确保 PRX 模式位正确
  uint8_t config = NRF_Force_Read_Reg(NRF24L01P_REG_CONFIG);
  config |= 0x70; // 屏蔽 MAX_RT / TX_DS / RX_DR 中断
  NRF_Force_Write_Reg(NRF24L01P_REG_CONFIG, config);

  // 5. 清空 FIFO 和状态标志
  nrf24l01p_flush_rx_fifo();
  nrf24l01p_flush_tx_fifo();
  NRF_Force_Write_Reg(NRF24L01P_REG_STATUS, 0x70);

  // 6. 预装填第一发 ACK 返回数据
  uint8_t init_ack[8] = {0xBB, 0, 0, 0, 0, 0, 0, 0};
  nrf24l01p_write_ack_payload(0, init_ack);

  // 7. 进入 PRX 模式（在地址写入之后调用，防止库函数覆盖地址）
  nrf24l01p_prx_mode();
}

// NRF通信恢复检查（每15秒检查一次）
void NRF_Recovery_Check(void) {
  if (nrf_last_rx_time > 0 && HAL_GetTick() - nrf_last_rx_time > 15000) {
    nrf_recovery_count++;
    if (nrf_recovery_count >= 2) {
      printf("[NRF] No RX for 30s, recovering...\r\n");
      // 调用完整配置函数，确保地址和所有寄存器被正确重载
      // 不再单独调用 nrf24l01p_prx_mode()，防止其覆盖配对地址
      NRF_Apply_Safe_Config();
      nrf_recovery_count = 0;
      nrf_last_rx_time = HAL_GetTick();
    }
  }
}

// 记录收到数据（每次收到数据时调用）
void NRF_OnDataReceived(void) {
  nrf_last_rx_time = HAL_GetTick();
  nrf_recovery_count = 0;
}

// ===================== 结束 NRF监控 =====================

void nrf24l01p_print_status(void)
{
  uint8_t status = nrf24l01p_get_status();
  uint8_t config = NRF_Force_Read_Reg(NRF24L01P_REG_CONFIG);
  uint8_t rf_ch = NRF_Force_Read_Reg(NRF24L01P_REG_RF_CH);
  uint8_t setup_aw = NRF_Force_Read_Reg(NRF24L01P_REG_SETUP_AW);
  uint8_t en_aa = NRF_Force_Read_Reg(NRF24L01P_REG_EN_AA);
  uint8_t en_rxaddr = NRF_Force_Read_Reg(NRF24L01P_REG_EN_RXADDR);
  uint8_t rx_addr[5], tx_addr[5];

  nrf24l01p_read_registers(NRF24L01P_REG_RX_ADDR_P0, rx_addr, 5);
  nrf24l01p_read_registers(NRF24L01P_REG_TX_ADDR, tx_addr, 5);

  printf("[NRF DEBUG] STATUS=0x%02X CONFIG=0x%02X RF_CH=%u EN_AA=0x%02X EN_RXADDR=0x%02X SETUP_AW=0x%02X\n",
         status, config, rf_ch, en_aa, en_rxaddr, setup_aw);
  printf("[NRF DEBUG] RX_ADDR_P0=%02X%02X%02X%02X%02X TX_ADDR=%02X%02X%02X%02X%02X\n",
         rx_addr[0], rx_addr[1], rx_addr[2], rx_addr[3], rx_addr[4],
         tx_addr[0], tx_addr[1], tx_addr[2], tx_addr[3], tx_addr[4]);
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET); // 强制拉高 SCL/SDA
  HAL_Delay(100);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  __HAL_RCC_AFIO_CLK_ENABLE();
  // 【黄金上电延时】
  // 等待电源电压彻底稳定，等待 NRF24L01 和 INA226 芯片完成内部上电复位
  HAL_Delay(200);
  // OLED 初始化（之前未调用，导致屏幕黑屏）
  OLED_Init();
  OLED_Clear();
  printf("\r\n--- Power RX Board Listening ---\r\n");

  // ID 显示现在由 INA226_ShowPower 统一管理

  // 1. 初始化 NRF (1Mbps)
  nrf24l01p_rx_init(2500, _1Mbps);

  // 2. 调用统一安全配置函数（初始化与恢复共用同一套逻辑，保证行为一致）
  NRF_Apply_Safe_Config();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  RX_Beep(100);

  if (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
    __HAL_RCC_I2C2_FORCE_RESET();
    HAL_Delay(2);
    __HAL_RCC_I2C2_RELEASE_RESET();
    MX_I2C2_Init();
  }
  INA226_Init();

  uint8_t rx_data[8];
  uint32_t power_counter = 0;
  float global_power_f = 0.0f;

  // ===== 添加物理按键(PA2)状态变量 =====
  uint8_t  button_state = 0;
  uint32_t button_press_time = 0;
  uint8_t  manual_lock_off = 0; // 0: 允许TX控制通断, 1: 物理按键强制关断，拒收TX开机指令
  // ===================================

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);

    // ===================== 物理按键逻辑 (PA2) =====================
    uint8_t current_pa2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);

    if (current_pa2 == GPIO_PIN_RESET) {
      // 按键被按下 (低电平有效)
      if (button_state == 0) {
        button_state = 1;
        button_press_time = HAL_GetTick(); // 记录刚刚按下的时刻
      } else if (button_state == 1) {
        // 人性化提示：按满2秒时，给一个短促提示音，告诉用户达到长按条件可以松手了
        if (HAL_GetTick() - button_press_time >= 2000) {
          button_state = 2; // 标记为已触发长按时间
          if (!beep_active) {
            RX_Beep_Start(50);
          }
        }
      }
    } else {
      // 按键松开 (高电平)
      if (button_state == 1 || button_state == 2) {
        uint32_t press_duration = HAL_GetTick() - button_press_time;
        button_state = 0; // 重置按键状态

        // 消除抖动 (>30ms 才认为是有效按键)
        if (press_duration > 30) {
          if (press_duration < 2000) {
            // 短按 (<2秒)松开：关机 (PB5 = 低电平)
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            manual_lock_off = 1; // 【强制锁定】禁止TX将其开机
            printf("[BUTTON] Short Press -> Power OFF\r\n");
            // 立即停止其他所有蜂鸣
            if (beep_active) {
              HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
              beep_active = 0;
              beep_sequence_step = 0;
            }
            RX_Beep_Shutdown_Sequence(); // 关机专属提示音
          } else {
            // 长按 (>=2秒)松开：开机 (PB5 = 高电平)
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            manual_lock_off = 0;
            printf("[BUTTON] Long Press -> Power ON\r\n");
            if (!beep_active) {
              RX_Beep_Start(100); // 开机提示音
            }
          }
        }
      }
    }
    // ===============================================================

    // NRF通信监控（状态检查、健康检查、通信恢复）
    NRF_Status_Check();
    NRF_Health_Check();
    NRF_Recovery_Check();

    uint8_t status = NRF_Force_Read_Reg(NRF24L01P_REG_STATUS);
    // 过滤 SPI 异常值：0xFF 表示总线故障（全高），0x00 表示总线故障（全低）
    // 两者都可能导致误判为"有数据"，从而刷新健康时间、阻止恢复机制触发
    if (status != 0xFF && status != 0x00 && (status & 0x40)) {
      // 不调用带中断控制的 rx_receive，直接使用最底层的读取
      nrf24l01p_read_rx_fifo(rx_data);

      // 记录收到数据
      NRF_OnDataReceived();
      
      // 打印 RX 收到的 TX 发送数据
      printf("[RX] Recv DATA: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
             rx_data[0], rx_data[1], rx_data[2], rx_data[3],
             rx_data[4], rx_data[5], rx_data[6], rx_data[7]);

      NRF_Force_Write_Reg(NRF24L01P_REG_STATUS, 0x70); // 清空标志位
      nrf24l01p_flush_rx_fifo();

      // 乘 1000，把单位变成毫瓦 (mW)
      // 比如 0.020W * 1000 = 20mW。这样强转整数就不会丢
      uint16_t pwr_int = (uint16_t)(global_power_f * 1000.0f);
      // 获取最新 MOS 状态
      uint8_t current_mos = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
      // 把 mos_state 塞到第 4 个位置（也就是索引 [3]）
      uint8_t ack_payload[8] = {0xBB, (uint8_t)(pwr_int >> 8), (uint8_t)(pwr_int & 0xFF), current_mos, 0, 0, 0, 0};

      if ((NRF_Force_Read_Reg(NRF24L01P_REG_FIFO_STATUS) & 0x20) == 0) {
        nrf24l01p_write_ack_payload(0, ack_payload);
      }

      if (rx_data[0] == 0xAA) {
        // 解析TX电池电压 (tx_data[1]是高8位，tx_data[2]是低8位，单位0.1V)
        tx_battery_voltage = ((uint16_t)rx_data[1] << 8) | rx_data[2];
        printf("[RX] TX Battery Voltage: %d.%dV\r\n", tx_battery_voltage / 10, tx_battery_voltage % 10);
        
        // 检查TX电池电压是否低于7.5V (75 = 7.5V)
        if (tx_battery_voltage > 0 && tx_battery_voltage < 75) {
          tx_low_voltage_alarm = 1;
        } else {
          tx_low_voltage_alarm = 0;
        }

        // ================= 修复 =================
        // 只有在【状态确实发生改变】时，才发出动作和蜂鸣器响声
        // 命令优先级处理：关机命令最高优先级
        if (rx_data[3] == 0x00)
        {
          // 关机命令
          // TX 试图关机：只有当前是开机状态，才需要动作！
          if (current_mos == GPIO_PIN_SET) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            // 注意：这里不再主动重置最大电流，以便断电记忆功能正常工作
            // 如果需要每次关机都重置最大电流，可以取消下面这行的注释：
            // INA226_ResetMaxCurrent();
            // 立即停止任何正在进行的蜂鸣
            if (beep_active) {
              HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
              beep_active = 0;
              beep_sequence_step = 0;
            }
            // 关机提示音：两次短响序列
            RX_Beep_Shutdown_Sequence();
            printf("[RX] Shutdown command executed (high priority)\r\n");
          }
        }
        else if (rx_data[3] == 0x01) {
          // TX 试图开机：必须同时满足“未被锁定” 且 “当前是关机状态”，才需要动作！
          if (manual_lock_off) {
            // 【核心逻辑】如果物理按键已经将系统锁定关机，则拒绝执行遥控器的开机指令！
            printf("[RX] Ignored TX Power ON (Manually LOCKED OFF)\r\n");
          } else {
            if (current_mos == GPIO_PIN_RESET) {
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
              if (!beep_active) {
                RX_Beep_Start(100);
              }
              printf("[RX] TX Power ON executed\r\n");
            }
          }
        }
      }
    }

    // ===== TX电池低压报警逻辑 (两短一长) =====
    if (tx_low_voltage_alarm) {
      alarm_beep_counter++;
      // 报警周期：约 600ms 一次两短一长
      // 两短一长：50ms + 50ms(停) + 50ms + 200ms = 350ms 响，周期约600ms
      uint32_t beep_pos = alarm_beep_counter % 12;  // 12*50ms = 600ms 周期
      
      // 两短：第1个50ms和第3个50ms，第2个和第4个50ms是间隔
      // 一长：第5-9个50ms (共250ms)
      if (beep_pos == 0 || beep_pos == 2) {
        // 短响 - 只有在没有其他蜂鸣时才响
        if (!beep_active) {
          HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
        }
      } else if (beep_pos >= 4 && beep_pos < 9) {
        // 长响：第5-9个50ms (共250ms) - 只有在没有其他蜂鸣时才响
        if (!beep_active) {
          HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
        }
      } else {
        // 只有在没有其他蜂鸣时才停止
        if (!beep_active) {
          HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        }
      }
    } else {
      // 清除报警状态，关闭蜂鸣器（只有在没有其他蜂鸣时才停止）
      if (!beep_active) {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
      }
      alarm_beep_counter = 0;
    }

    if (++power_counter >= 20)
    {
      power_counter = 0;
      INA226_ShowPower(NRF_PAIR_ID);
      global_power_f = INA226_ReadPower();

      // 动态 I2C2 恢复：INA226_ShowPower 检测到运行时读取失败后会将 ina226_ready 置 0
      // 此处捕获该状态，对 I2C2 总线执行软件复位并重新初始化 INA226
      if (!INA226_GetStatus()) {
        printf("[I2C2] INA226 error detected, resetting I2C2 bus...\r\n");
        __HAL_RCC_I2C2_FORCE_RESET();
        HAL_Delay(5);
        __HAL_RCC_I2C2_RELEASE_RESET();
        HAL_Delay(5);
        MX_I2C2_Init();
        INA226_Init(); // 重新配置 INA226 寄存器（含 CONFIG 和 CALIB）
      }
      // 【修复】将静态变量定义在 if...else 外部，保证是同一个变量
      static uint8_t over_power_count = 0;
      if (global_power_f > 30.0f) {
        // 非阻塞蜂鸣，避免系统阻塞
        if (!beep_active) {
          RX_Beep_Start(500);
        }
        
        // 紧急情况：如果功率持续过大，考虑自动保护
        over_power_count++;
        if (over_power_count > 10) { // 连续10次检测到过功率（约10秒）
          // 自动关闭输出作为保护
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
          printf("[EMERGENCY] Auto shutdown due to sustained over-power!\r\n");
          over_power_count = 0; // 触发后清零
        }
      } else {
        // 功率恢复正常，重置过功率计数
        over_power_count = 0;
      }
    }
    
    // 更新非阻塞蜂鸣器状态
    RX_Beep_Update();
    
    HAL_Delay(50); // 适度轮询延时，保持极高的按键响应灵敏度
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
