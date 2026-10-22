#include "usart1.h"
#include <string.h>

// 静态变量（只在本文件使用）
static uint8_t rx_buf[32];
static uint8_t rx_idx = 0;
static uint8_t rx_in_packet = 0;

// 全局变量定义（供外部使用）
volatile uint8_t usart1_cmd_ready = 0;
char usart1_cmd_char = 0;
float usart1_cmd_value = 0;

void USART1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    
    // PA9 TX, PA10 RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART1);
        
        if (data == '{') {
            rx_in_packet = 1;
            rx_idx = 0;
            memset(rx_buf, 0, sizeof(rx_buf));
        }
        else if (data == '}') {
            rx_in_packet = 0;
            usart1_cmd_ready = 1;
        }
        else if (rx_in_packet) {
            if (rx_idx < sizeof(rx_buf) - 1) {
                rx_buf[rx_idx++] = data;
            }
        }
        
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void USART1_CheckAndParse(void)
{
    float val;
    
    if (rx_idx == 0) return;
    
    // 命令字：第一个字符
    usart1_cmd_char = rx_buf[0];
    
    // 解析数值（如果有）
    if (rx_idx >= 2) {
        uint8_t i = 1;
        int integer = 0;
        float fraction = 0;
        uint8_t is_negative = 0;
        uint8_t is_fraction = 0;
        float fraction_div = 1.0f;
        
        if (rx_buf[i] == '-') {
            is_negative = 1;
            i++;
        }
        
        for (; i < rx_idx; i++) {
            if (rx_buf[i] >= '0' && rx_buf[i] <= '9') {
                if (is_fraction) {
                    fraction_div *= 10;
                    fraction = fraction + (float)(rx_buf[i] - '0') / fraction_div;
                } else {
                    integer = integer * 10 + (rx_buf[i] - '0');
                }
            } else if (rx_buf[i] == '.') {
                is_fraction = 1;
            }
        }
        
        val = (float)integer + fraction;
        if (is_negative) val = -val;
        usart1_cmd_value = val;
    } else {
        usart1_cmd_value = 0;
    }
		// 解析完成，清除缓冲区
    rx_idx = 0;
    memset(rx_buf, 0, sizeof(rx_buf));
}

