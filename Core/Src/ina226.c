#include "ina226.h"
#include "OLED.h"
#include <stdio.h>

/* 硬件参数配置 */
#define INA226_SHUNT_OHM     0.0001f    // 采样电阻 0.1mR
static float ina226_current_lsb = 0.0002f; // 电流步进 0.2mA
static float ina226_power_lsb   = 0.004f;  // 功率步进 (25 * Current_LSB)

/* 状态变量 */
static uint8_t ina226_ready = 0;
static float ina226_max_current = 0.0f;  // 记录历史最大电流 (A)
static uint8_t flash_initialized = 0;    // Flash 是否已初始化

/* 使用 I2C2 句柄 */
extern I2C_HandleTypeDef hi2c2;

uint8_t INA226_GetStatus(void) {
    return ina226_ready;
}

/* 内部寄存器写入函数 */
HAL_StatusTypeDef INA226_WriteReg(uint8_t reg, uint16_t value)
{
    uint8_t buf[2] = {(uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return HAL_I2C_Mem_Write(&hi2c2, INA226_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 200);
}

/* 内部寄存器读取函数 */
HAL_StatusTypeDef INA226_ReadReg(uint8_t reg, uint16_t *value)
{
    uint8_t tmp[2] = {0};
    if (HAL_I2C_Mem_Read(&hi2c2, INA226_ADDR, reg, I2C_MEMADD_SIZE_8BIT, tmp, 2, 200) != HAL_OK)
        return HAL_ERROR;

    *value = (uint16_t)((tmp[0] << 8) | tmp[1]);
    return HAL_OK;
}

/**
  * @brief  INA226 初始化
  */
HAL_StatusTypeDef INA226_Init(void)
{
    uint16_t id = 0;
    HAL_StatusTypeDef ret;

    printf("INA226_Init: start\r\n");

    // 读取 ID 寄存器（0xFE）
    ret = HAL_I2C_Mem_Read(&hi2c2, INA226_ADDR, INA226_REG_ID, I2C_MEMADD_SIZE_8BIT,
                           (uint8_t*)&id, 2, 50);
    printf("Read ID: ret=%d, id=0x%04X\r\n", ret, id);

    if (ret != HAL_OK) {
        printf("INA226: read ID failed! (maybe address wrong or no device)\r\n");
        ina226_ready = 0;
        return HAL_ERROR;
    }

    // 写入配置寄存器
    ret = INA226_WriteReg(INA226_REG_CONFIG, 0x4127);
    printf("Write CONFIG: ret=%d\r\n", ret);

    // 写入校准寄存器
    ret = INA226_WriteReg(INA226_REG_CALIB, 5120);
    printf("Write CALIB: ret=%d\r\n", ret);

    ina226_ready = 1;
    printf("INA226_Init: success\r\n");
    
    // 初始化后从 Flash 加载历史最大电流
    INA226_LoadMaxCurrentFromFlash();
    
    return HAL_OK;
}

/**
  * @brief  擦除 Flash 页
  */
HAL_StatusTypeDef INA226_EraseFlashPage(void)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;
    
    // 解锁 Flash
    HAL_FLASH_Unlock();
    
    // 配置擦除参数
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = FLASH_USER_START_ADDR;
    erase_init.NbPages = 1;  // 只擦除1页
    
    // 执行擦除
    if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK) {
        HAL_FLASH_Lock();
        printf("Flash erase failed!\r\n");
        return HAL_ERROR;
    }
    
    HAL_FLASH_Lock();
    printf("Flash page erased at 0x%08X\r\n", FLASH_USER_START_ADDR);
    return HAL_OK;
}

/**
  * @brief  保存最大电流到 Flash
  */
HAL_StatusTypeDef INA226_SaveMaxCurrentToFlash(void)
{
    FlashData_t flash_data;
    
    // 准备数据
    flash_data.magic = FLASH_MAGIC_NUMBER;
    flash_data.version = FLASH_DATA_VERSION;
    flash_data.max_current = ina226_max_current;
    flash_data.crc32 = 0;  // 简单实现，不计算CRC
    
    // 先擦除
    if (INA226_EraseFlashPage() != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 解锁 Flash
    HAL_FLASH_Unlock();
    
    // 写入数据（按32位写入）
    uint32_t *data_ptr = (uint32_t*)&flash_data;
    uint32_t addr = FLASH_USER_START_ADDR;
    
    for (uint8_t i = 0; i < sizeof(FlashData_t) / 4; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data_ptr[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            printf("Flash write failed at 0x%08X\r\n", addr);
            return HAL_ERROR;
        }
        addr += 4;
    }
    
    HAL_FLASH_Lock();
    printf("Max current saved to Flash: %.4fA\r\n", ina226_max_current);
    return HAL_OK;
}

/**
  * @brief  从 Flash 加载最大电流
  */
HAL_StatusTypeDef INA226_LoadMaxCurrentFromFlash(void)
{
    FlashData_t *flash_data = (FlashData_t*)FLASH_USER_START_ADDR;
    
    // 检查魔数和版本
    if (flash_data->magic != FLASH_MAGIC_NUMBER) {
        printf("Flash data invalid (magic: 0x%08X)\r\n", flash_data->magic);
        ina226_max_current = 0.0f;
        return HAL_ERROR;
    }
    
    if (flash_data->version != FLASH_DATA_VERSION) {
        printf("Flash data version mismatch (got %d, expected %d)\r\n",
               flash_data->version, FLASH_DATA_VERSION);
        ina226_max_current = 0.0f;
        return HAL_ERROR;
    }
    
    // 加载数据
    ina226_max_current = flash_data->max_current;
    printf("Max current loaded from Flash: %.4fA\r\n", ina226_max_current);
    flash_initialized = 1;
    return HAL_OK;
}

/**
  * @brief  读取总线电压 (单位: V)
  */
float INA226_ReadVoltage(void)
{
    uint16_t raw;
    if (INA226_ReadReg(INA226_REG_BUSVOLT, &raw) != HAL_OK)
        return -1.0f;

    return (float)raw * 0.00125f; // INA226 总线电压固定 LSB 为 1.25mV
}

/**
  * @brief  读取电流 (单位: A)，并自动更新历史最大电流
  */
float INA226_ReadCurrent(void)
{
    uint16_t raw;
    if (INA226_ReadReg(INA226_REG_CURRENT, &raw) != HAL_OK)
        return -1.0f;

    float current = (float)((int16_t)raw) * ina226_current_lsb;

    // 只记录正向电流的最大值
    if (current > ina226_max_current) {
        ina226_max_current = current;
        
        // 只有当电流显著增加时才保存到 Flash（避免频繁写）
        static float last_saved_current = 0.0f;
        if (ina226_max_current - last_saved_current > 0.1f) {  // 变化超过 0.1A 才保存
            INA226_SaveMaxCurrentToFlash();
            last_saved_current = ina226_max_current;
        }
    }

    return current;
}

/**
  * @brief  获取历史最大电流 (单位: A)
  */
float INA226_GetMaxCurrent(void)
{
    return ina226_max_current;
}

/**
  * @brief  重置历史最大电流记录
  */
void INA226_ResetMaxCurrent(void)
{
    ina226_max_current = 0.0f;
    INA226_SaveMaxCurrentToFlash();  // 保存零值到 Flash
}

/**
  * @brief  读取功率 (单位: W)
  */
float INA226_ReadPower(void)
{
    uint16_t raw;
    if (INA226_ReadReg(INA226_REG_POWER, &raw) != HAL_OK)
        return -1.0f;

    return (float)raw * ina226_power_lsb;
}

/**
  * @brief  OLED 两行显示：
  *         第1行: x max:xx.xxA  (ID只显示数字，最大电流2位小数)
  *         第2行: xx.xxxW  xx.xxxV  (功率和电压都2位小数)
  * @param  id      设备ID编号
  */
void INA226_ShowPower(uint8_t id)
{
    char buf[17];

    if (!ina226_ready) {
        OLED_ShowString(1, 1, "INA226  ERR     ");
        OLED_ShowString(2, 1, "                ");
        return;
    }

    float v = INA226_ReadVoltage();
    float i = INA226_ReadCurrent();  // 内部自动更新最大电流
    float p = INA226_ReadPower();

    // 运行时 I2C 故障检测：任意一路读取返回 -1.0f 即视为总线异常
    // 将 ina226_ready 置 0，通知主循环执行动态恢复
    if (v < 0.0f || i < -0.5f || p < 0.0f) {
        ina226_ready = 0;
        OLED_ShowString(1, 1, "INA226  ERR     ");
        OLED_ShowString(2, 1, "                ");
        return;
    }

    printf("DEBUG: V=%.3fV  I=%.4fA  P=%.4fW  Imax=%.4fA\n",
           v, i, p, ina226_max_current);

    /* --- 第1行: ID数字 + 最大电流 --- */
    char line1[20];
    
    // 最大电流保留2位小数
    int imax_cA = (int)(ina226_max_current * 100 + 0.5f);  // 单位 0.01A
    
    // 格式: "0 max:10.29A" (12字符)
    sprintf(line1, "  %d   MAX:%2d.%02dA",
            id,
            imax_cA / 100, imax_cA % 100);
    
    OLED_ShowString(1, 1, line1);

    /* --- 第2行: 功率 + 电压 (都2位小数) --- */
    char line2[20];
    
    // 功率保留2位小数
    int p_cW = (int)(p * 100 + 0.5f);    // 单位 0.01W
    
    // 电压保留2位小数
    int v_cV = (int)(v * 100 + 0.5f);    // 单位 0.01V
    
    // 格式: "10.10W  24.22V" (14字符)
    sprintf(line2, "%2d.%03dW  %2d.%03dV",
            p_cW / 100, p_cW % 100,
            v_cV / 100, v_cV % 100);
    
    OLED_ShowString(2, 1, line2);
}