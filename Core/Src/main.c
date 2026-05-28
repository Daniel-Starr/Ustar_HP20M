/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : HPTM100 RS485 透传桥接
  *
  * @description    : 从 USART1 接收 HEX 命令，转发到 USART3 (RS485)，
  *                   收到传感器响应后，原样通过 USART1 返回 HEX 数据。
  *
  * 使用方法：
  *   在串口助手中选择 HEX 发送模式，发送 Modbus 命令，
  *   即可在接收区看到传感器的 HEX 响应。
  *
  *   发送：01 03 04 04 00 02 84 FA  →  接收压力数据
  *   发送：01 03 04 08 00 02 05 3A  →  接收温度数据
  *
  * 硬件连接：
  *   USART1 (PA9-TX / PA10-RX) — USB转TTL → 电脑
  *   USART3 (PA7-TX / PA5-RX)  — RS485自动方向模块(5V) → HPTM100
  ******************************************************************************
  */

#include "main.h"
#include <string.h>

/* ========================== 宏定义 ========================== */

#define CMD_BUF_SIZE            16          /* 命令缓冲区 */
#define RX_BUF_SIZE             24          /* 响应缓冲区 (回环+响应) */
#define CMD_TIMEOUT_MS          100         /* 接收命令字节间超时 */
#define FIRST_BYTE_TIMEOUT_MS   500         /* 等待传感器响应第一个字节 */
#define INTER_BYTE_TIMEOUT_MS   50          /* 响应字节间超时 */
#define MODBUS_RESP_LEN         9           /* 标准响应帧长度 */

/* ========================== 全局变量 ========================== */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

static uint8_t cmd_buf[CMD_BUF_SIZE];       /* 接收到的命令 */
static uint8_t rx_buf[RX_BUF_SIZE];         /* 传感器响应 */

/* ========================== 函数声明 ========================== */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);

/* ========================== USART3 底层 ========================== */

static void UART3_Reset(void)
{
    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_OREF);
    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_FEF);
    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_NEF);
    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_PEF);
    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_IDLEF);

    volatile uint32_t dummy;
    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE)) {
        dummy = huart3.Instance->RDR;
        (void)dummy;
    }

    huart3.RxState   = HAL_UART_STATE_READY;
    huart3.gState    = HAL_UART_STATE_READY;
    huart3.ErrorCode = HAL_UART_ERROR_NONE;
}

/**
  * @brief  逐字节接收直到总线空闲
  */
static uint16_t UART3_ReceiveAll(uint8_t *buf, uint16_t max_len)
{
    uint16_t count = 0;
    for (uint16_t i = 0; i < max_len; i++) {
        uint32_t timeout = (count == 0) ? FIRST_BYTE_TIMEOUT_MS : INTER_BYTE_TIMEOUT_MS;

        if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE))
            __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_OREF);
        huart3.RxState   = HAL_UART_STATE_READY;
        huart3.ErrorCode = HAL_UART_ERROR_NONE;

        if (HAL_UART_Receive(&huart3, &buf[i], 1, timeout) == HAL_OK)
            count++;
        else
            break;
    }
    return count;
}

/* ========================== Modbus CRC ========================== */

static uint16_t CRC16_Modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else { crc >>= 1; }
        }
    }
    return crc;
}

/**
  * @brief  从接收数据中提取有效 Modbus 响应帧 (跳过回环)
  * @param  buf:      全部接收数据
  * @param  len:      数据长度
  * @param  resp:     输出响应帧
  * @param  resp_len: 输出响应帧长度
  * @retval 0=成功, -1=未找到
  */
static int ExtractResponse(const uint8_t *buf, uint16_t len,
                            uint8_t *resp, uint16_t *resp_len)
{
    /* 搜索 [地址] [0x03] [字节数N] [N字节数据] [CRC低] [CRC高] */
    for (uint16_t i = 0; i + 5 <= len; i++) {
        if (buf[i] >= 0x01 && buf[i] <= 0xF7 && buf[i + 1] == 0x03) {
            uint8_t byte_count = buf[i + 2];
            uint16_t frame_len = 3 + byte_count + 2;  /* 地址+功能码+字节数+数据+CRC */

            if (i + frame_len <= len && byte_count > 0 && byte_count <= 20) {
                uint16_t crc_calc = CRC16_Modbus(&buf[i], 3 + byte_count);
                uint16_t crc_recv = (uint16_t)buf[i + frame_len - 1] << 8 | buf[i + frame_len - 2];

                if (crc_calc == crc_recv) {
                    memcpy(resp, &buf[i], frame_len);
                    *resp_len = frame_len;
                    return 0;
                }
            }
        }
    }
    return -1;
}

/* ========================== 主程序 ========================== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART3_UART_Init();
    MX_USART1_UART_Init();

    /* 禁用 USART3 中断 */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
    UART3_Reset();

    HAL_Delay(3000);   /* 等待传感器上电 */

    while (1)
    {
        /*
         * 第1步：等待从 USART1 接收 HEX 命令
         *        逐字节接收，字节间超时 CMD_TIMEOUT_MS
         *        超时即认为一帧命令接收完毕
         */
        uint16_t cmd_len = 0;
        memset(cmd_buf, 0, sizeof(cmd_buf));

        /* 等待第一个字节 (无限等待) */
        if (HAL_UART_Receive(&huart1, &cmd_buf[0], 1, HAL_MAX_DELAY) == HAL_OK) {
            cmd_len = 1;

            /* 继续接收后续字节 */
            for (uint16_t i = 1; i < CMD_BUF_SIZE; i++) {
                if (HAL_UART_Receive(&huart1, &cmd_buf[i], 1, CMD_TIMEOUT_MS) == HAL_OK)
                    cmd_len++;
                else
                    break;
            }
        }

        if (cmd_len == 0)
            continue;

        /*
         * 第2步：转发命令到 USART3 (RS485 → 传感器)
         */
        UART3_Reset();
        memset(rx_buf, 0, sizeof(rx_buf));

        HAL_UART_Transmit(&huart3, cmd_buf, cmd_len, 200);

        /* 等待发送完毕 */
        uint32_t tick = HAL_GetTick();
        while (!__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC)) {
            if ((HAL_GetTick() - tick) > 100) break;
        }

        /*
         * 第3步：接收所有返回数据 (回环 + 传感器响应)
         */
        uint16_t total = UART3_ReceiveAll(rx_buf, RX_BUF_SIZE);

        /*
         * 第4步：提取有效响应帧，通过 USART1 返回给电脑
         */
        if (total > 0) {
            uint8_t resp[RX_BUF_SIZE];
            uint16_t resp_len = 0;

            if (ExtractResponse(rx_buf, total, resp, &resp_len) == 0) {
                /* 只返回有效响应帧的 HEX 原始数据 */
                HAL_UART_Transmit(&huart1, resp, resp_len, 200);
            }
        }
    }
}

/* ========================== 系统配置 ========================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
        Error_Handler();

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_4;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                      | RCC_CLOCKTYPE_PCLK3;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance                    = USART1;
    huart1.Init.BaudRate               = 9600;
    huart1.Init.WordLength             = UART_WORDLENGTH_8B;
    huart1.Init.StopBits               = UART_STOPBITS_1;
    huart1.Init.Parity                 = UART_PARITY_NONE;
    huart1.Init.Mode                   = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) Error_Handler();
}

static void MX_USART3_UART_Init(void)
{
    huart3.Instance                    = USART3;
    huart3.Init.BaudRate               = 9600;
    huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    huart3.Init.StopBits               = UART_STOPBITS_1;
    huart3.Init.Parity                 = UART_PARITY_NONE;
    huart3.Init.Mode                   = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
    if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif