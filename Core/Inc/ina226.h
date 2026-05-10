#ifndef __INA226_H
#define __INA226_H

#include "main.h"
#include "i2c.h"
#include "stm32f1xx_hal_flash.h"

/* INA226 硬件地址 */
#define INA226_ADDR             (0x40 << 1)

/* INA226 寄存器地址定义 (严格对应 Datasheet) */
#define INA226_REG_CONFIG       0x00    // 配置寄存器
#define INA226_REG_SHUNTV       0x01    // 分流电压 (Shunt Voltage)
#define INA226_REG_BUSVOLT      0x02    // 总线电压 (Bus Voltage)
#define INA226_REG_POWER        0x03    // 功率
#define INA226_REG_CURRENT      0x04    // 电流
#define INA226_REG_CALIB        0x05    // 校准寄存器
#define INA226_REG_MASK         0x06    // 屏蔽/使能寄存器
#define INA226_REG_ALERT        0x07    // 警报限值寄存器
#define INA226_REG_ID           0xFE    // 厂商 ID (应为 0x5449)

/* Flash 存储配置 (STM32F103 64KB Flash) */
#define FLASH_USER_START_ADDR   ((uint32_t)0x0800F000)  // 最后1KB空间
#define FLASH_USER_END_ADDR     ((uint32_t)0x0800FFFF)
#define FLASH_PAGE_SIZE         1024  // STM32F103 1KB 每页
#define FLASH_MAGIC_NUMBER      0x55AA1234
#define FLASH_DATA_VERSION      0x01

// Flash 数据结构
typedef struct {
    uint32_t magic;          // 魔数，用于验证数据有效性
    uint32_t version;        // 数据结构版本
    float max_current;       // 历史最大电流 (A)
    uint32_t crc32;          // 校验和（可选）
} FlashData_t;

// 函数声明
HAL_StatusTypeDef INA226_Init(void);
float INA226_ReadVoltage(void);
float INA226_ReadCurrent(void);
float INA226_ReadPower(void);
float INA226_GetMaxCurrent(void);
void  INA226_ResetMaxCurrent(void);
void INA226_ShowPower(uint8_t id);
uint8_t INA226_GetStatus(void);

// Flash 存储相关函数
HAL_StatusTypeDef INA226_SaveMaxCurrentToFlash(void);
HAL_StatusTypeDef INA226_LoadMaxCurrentFromFlash(void);
HAL_StatusTypeDef INA226_EraseFlashPage(void);

#endif