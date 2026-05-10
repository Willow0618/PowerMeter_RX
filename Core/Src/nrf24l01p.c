/*
 *  nrf24l01_plus.c
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 * 
 */


#include "nrf24l01p.h"
#include "ina226.h"
#include "main.h" // for HAL_Delay
#include <stdio.h>

extern void RX_Beep(uint16_t duration_ms);
extern void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);

#include <string.h>
extern SPI_HandleTypeDef hspi1;

static void cs_high()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_SET);
}

static void cs_low()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_RESET);
}

static void ce_high()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_SET);
}

static void ce_low()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_RESET);
}

// 替换掉原作者的 read_register
static uint8_t read_register(uint8_t reg)
{
    // 读指令是 NRF24L01P_CMD_R_REGISTER (0x00)
    uint8_t tx_buf[2] = { NRF24L01P_CMD_R_REGISTER | reg, 0xFF };
    uint8_t rx_buf[2] = { 0, 0 };
    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, tx_buf, rx_buf, 2, 2000); // 连续边发边收
    cs_high();
    return rx_buf[1];
}

// 替换掉原作者的 write_register
static uint8_t write_register(uint8_t reg, uint8_t value)
{
    // 写指令是 NRF24L01P_CMD_W_REGISTER (0x20)
    uint8_t tx_buf[2] = { NRF24L01P_CMD_W_REGISTER | reg, value };
    cs_low();
    HAL_SPI_Transmit(NRF24L01P_SPI, tx_buf, 2, 2000); // 连续发
    cs_high();
    return value;
}

// static uint8_t read_register(uint8_t reg)
// {
//     uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
//     uint8_t status;
//     uint8_t read_val;
//
//     cs_low();
//     HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
//     HAL_SPI_Receive(NRF24L01P_SPI, &read_val, 1, 2000);
//     cs_high();
//
//     return read_val;
// }
//
// static uint8_t write_register(uint8_t reg, uint8_t value)
// {
//     uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
//     uint8_t status;
//     uint8_t write_val = value;
//
//     cs_low();
//     HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
//     HAL_SPI_Transmit(NRF24L01P_SPI, &write_val, 1, 2000);
//     cs_high();
//
//     return write_val;
// }


/* nRF24L01+ Main Functions */
void nrf24l01p_rx_init(channel MHz, air_data_rate bps)
{
    nrf24l01p_reset();

    nrf24l01p_prx_mode();
    nrf24l01p_power_up();

    nrf24l01p_rx_set_payload_widths(NRF24L01P_PAYLOAD_LENGTH);
    // 强制设置通道 0 的 Payload 为 8 字节！
    write_register(NRF24L01P_REG_RX_PW_P0, 8);
    // 地址会在 main.c 中通过 NRF_Force_Set_Addr 设置

    // uint8_t addr[5] = {0x66, 0x77, 0x88, 0x99, 0xAA};
    // // ======== 添加下面这一行 ========
    // nrf24l01p_set_addr(NRF24L01P_REG_RX_ADDR_P0, addr); // 将地址真实写进 Pipe0

    nrf24l01p_set_rf_channel(MHz);
    nrf24l01p_set_rf_air_data_rate(bps);
    nrf24l01p_set_rf_tx_output_power(_0dBm);

    nrf24l01p_set_crc_length(1);
    nrf24l01p_set_address_widths(5);

    nrf24l01p_auto_retransmit_count(3);
    nrf24l01p_auto_retransmit_delay(250);
    
    ce_high();
}

void nrf24l01p_tx_init(channel MHz, air_data_rate bps)
{
    nrf24l01p_reset();

    nrf24l01p_ptx_mode();
    nrf24l01p_power_up();

    nrf24l01p_set_rf_channel(MHz);
    nrf24l01p_set_rf_air_data_rate(bps);
    nrf24l01p_set_rf_tx_output_power(_0dBm);

    nrf24l01p_set_crc_length(1);
    nrf24l01p_set_address_widths(5);

    nrf24l01p_auto_retransmit_count(3);
    nrf24l01p_auto_retransmit_delay(250);

    ce_high();
}

void nrf24l01p_rx_receive(uint8_t* rx_payload)
{
    // 1. 先读出收到的数据
    nrf24l01p_read_rx_fifo(rx_payload);
    nrf24l01p_clear_rx_dr();

    // 处理开关指令
    if (rx_payload[0] == 0xAA) {
        uint8_t cmd = rx_payload[3];
        // ======= 关键修改：只处理 0x00 和 0x01，忽略 0xFF(心跳查询) =======
        if (cmd == 0x00 || cmd == 0x01) {
            static uint8_t last_switch_state = 0; // 0=关, 1=开
            uint8_t current_switch_state = cmd;

            if (current_switch_state != last_switch_state) {
                if (current_switch_state == 1) {
                    RX_Beep(100);
                    OLED_ShowString(1, 1, "MOS: ON ");
                } else {
                    RX_Beep(50);
                    HAL_Delay(50);
                    RX_Beep(50);
                    OLED_ShowString(1, 1, "MOS: OFF");
                }
                last_switch_state = current_switch_state;
            }
        }
    }


    // 4. 【关键修改】发完立刻切回接收模式
    nrf24l01p_clear_tx_ds();
    nrf24l01p_clear_max_rt();
    nrf24l01p_prx_mode(); // 切回接收
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // 开启监听
}

void nrf24l01p_tx_transmit(uint8_t* tx_payload)
{
    nrf24l01p_write_tx_fifo(tx_payload);
}

void nrf24l01p_tx_irq()
{
    uint8_t tx_ds = nrf24l01p_get_status();
    tx_ds &= 0x20;

    if(tx_ds)
    {   
        // TX_DS
        //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        nrf24l01p_clear_tx_ds();
    }

    else
    {
        // MAX_RT
        //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
        nrf24l01p_clear_max_rt();
    }
}

/* nRF24L01+ Sub Functions */
void nrf24l01p_reset()
{
    // Reset pins
    cs_high();
    ce_low();

    // Reset registers
    write_register(NRF24L01P_REG_CONFIG, 0x08);
    write_register(NRF24L01P_REG_EN_AA, 0x3F);
    write_register(NRF24L01P_REG_EN_RXADDR, 0x03);
    write_register(NRF24L01P_REG_SETUP_AW, 0x03);
    write_register(NRF24L01P_REG_SETUP_RETR, 0x03);
    write_register(NRF24L01P_REG_RF_CH, 0x02);
    write_register(NRF24L01P_REG_RF_SETUP, 0x07);
    write_register(NRF24L01P_REG_STATUS, 0x7E);
    write_register(NRF24L01P_REG_RX_PW_P0, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P0, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P1, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P2, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P3, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P4, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P5, 0x00);
    write_register(NRF24L01P_REG_FIFO_STATUS, 0x11);
    write_register(NRF24L01P_REG_DYNPD, 0x00);
    write_register(NRF24L01P_REG_FEATURE, 0x00);

    // Reset FIFO
    nrf24l01p_flush_rx_fifo();
    nrf24l01p_flush_tx_fifo();
}

void nrf24l01p_prx_mode()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 0;

    write_register(NRF24L01P_REG_CONFIG, new_config);
    // 【关键】开启 Pipe0 接收地址使能 (EN_RXADDR)
    uint8_t en_rxaddr = read_register(NRF24L01P_REG_EN_RXADDR);
    en_rxaddr |= 0x01; // 开启 Pipe0
    write_register(NRF24L01P_REG_EN_RXADDR, en_rxaddr);
}

void nrf24l01p_ptx_mode()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFE;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

uint8_t nrf24l01p_read_rx_fifo(uint8_t* rx_payload)
{
    uint8_t command = NRF24L01P_CMD_R_RX_PAYLOAD;
    uint8_t status;
    // 准备一个空数组用于发时钟
    uint8_t dummy[NRF24L01P_PAYLOAD_LENGTH];
    memset(dummy, 0xFF, NRF24L01P_PAYLOAD_LENGTH);

    cs_low();
    // 发送读命令并获取 STATUS
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    // 发送空字节的同时读回 payload (非常关键！)
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, dummy, rx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000);
    cs_high();

    return status;
}

uint8_t nrf24l01p_write_tx_fifo(uint8_t* tx_payload)
{
    uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, tx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000);
    cs_high(); 

    return status;
}

void nrf24l01p_write_ack_payload(uint8_t pipe, uint8_t* payload)
{
    uint8_t command = NRF24L01P_CMD_W_ACK_PAYLOAD | pipe;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, payload, NRF24L01P_PAYLOAD_LENGTH, 2000);
    cs_high();
}

void nrf24l01p_flush_rx_fifo()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_RX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

void nrf24l01p_flush_tx_fifo()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_TX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

uint8_t nrf24l01p_get_status()
{
    uint8_t command = NRF24L01P_CMD_NOP;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high(); 

    return status;
}

uint8_t nrf24l01p_get_fifo_status()
{
    return read_register(NRF24L01P_REG_FIFO_STATUS);
}

void nrf24l01p_read_registers(uint8_t reg, uint8_t* buf, uint8_t len)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;

    cs_low();
    HAL_SPI_Transmit(NRF24L01P_SPI, &command, 1, 2000);
    for (uint8_t i = 0; i < len; i++)
    {
        HAL_SPI_TransmitReceive(NRF24L01P_SPI, (uint8_t[]){0xFF}, &buf[i], 1, 2000);
    }
    cs_high();
}


void nrf24l01p_rx_set_payload_widths(widths bytes)
{
    write_register(NRF24L01P_REG_RX_PW_P0, bytes);
}

void nrf24l01p_clear_rx_dr()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x40;

    write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_clear_tx_ds()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x20;

    write_register(NRF24L01P_REG_STATUS, new_status);     
}

void nrf24l01p_clear_max_rt()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x10;

    write_register(NRF24L01P_REG_STATUS, new_status); 
}

void nrf24l01p_power_up()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 1;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_power_down()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFD;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_crc_length(length bytes)
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    
    switch(bytes)
    {
        // CRCO bit in CONFIG resiger set 0
        case 1:
            new_config &= 0xFB;
            break;
        // CRCO bit in CONFIG resiger set 1
        case 2:
            new_config |= 1 << 2;
            break;
    }

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_address_widths(widths bytes)
{
    write_register(NRF24L01P_REG_SETUP_AW, bytes - 2);
}

void nrf24l01p_auto_retransmit_count(count cnt)
{
    uint8_t new_setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);
    
    // Reset ARC register 0
    new_setup_retr |= 0xF0;
    new_setup_retr |= cnt;
    write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_auto_retransmit_delay(delay us)
{
    uint8_t new_setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);

    // Reset ARD register 0
    new_setup_retr |= 0x0F;
    new_setup_retr |= ((us / 250) - 1) << 4;
    write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_set_rf_channel(channel MHz)
{
	uint16_t new_rf_ch = MHz - 2400;
    write_register(NRF24L01P_REG_RF_CH, new_rf_ch);
}

void nrf24l01p_set_rf_tx_output_power(output_power dBm)
{
    uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xF9;
    new_rf_setup |= (dBm << 1);

    write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

void nrf24l01p_set_rf_air_data_rate(air_data_rate bps)
{
    // Set value to 0
    uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xD7;
    
    switch(bps)
    {
        case _1Mbps: 
            break;
        case _2Mbps: 
            new_rf_setup |= 1 << 3;
            break;
        case _250kbps:
            new_rf_setup |= 1 << 5;
            break;
    }
    write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

// 新增函数到 nrf24l01p.c
void nrf24l01p_set_addr(uint8_t reg, uint8_t* addr) {
    cs_low();
    uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
    HAL_SPI_Transmit(NRF24L01P_SPI, &command, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, addr, 5, 2000); // 写入 5 字节地址
    cs_high();
}

void NRF_Force_Write_Reg(uint8_t reg, uint8_t value) {
    // 必须加上 NRF24L01P_CMD_W_REGISTER (0x20)
    uint8_t buf[2] = { NRF24L01P_CMD_W_REGISTER | reg, value };
    cs_low();
    HAL_SPI_Transmit(NRF24L01P_SPI, buf, 2, HAL_MAX_DELAY); // 两字节一口气发完，防止时序断裂
    cs_high();
}

uint8_t NRF_Force_Read_Reg(uint8_t reg) {
    // 读指令是 NRF24L01P_CMD_R_REGISTER (0x00)
    uint8_t tx_buf[2] = { NRF24L01P_CMD_R_REGISTER | reg, 0xFF }; // 发送 dummy(0xFF) 换回真实数据
    uint8_t rx_buf[2] = { 0, 0 };
    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, tx_buf, rx_buf, 2, HAL_MAX_DELAY); // 边发边收
    cs_high();
    return rx_buf[1]; // 第二个字节才是我们要的寄存器值
}