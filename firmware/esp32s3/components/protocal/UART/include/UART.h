/*
 * UART1.h
 *
 *  Created on: 2022年11月14日
 *      Author: admin
 */

#ifndef _UART_H_
#define _UART_H_

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "ring_buffer.h"
#include "wireless_conn.h"

#define     LIDAR_UART_NUM   		 UART_NUM_1
#define     baud_R 					 115200
#define     UART1_TXD_PIN 			 17
#define     UART1_RXD_PIN 			 18
#define     UART1_RX0_BUF_SIZE       4096

#define     USER_UART_NUM   		 UART_NUM_0
#define 	UART0_TXD_PIN            43
#define 	UART0_RXD_PIN            44
#define 	UART0_RX0_BUF_SIZE       1024



typedef struct {
    uint8_t header;    // 0xAA
    char data[64];     // 数据部分
    uint8_t tail;      // 0x55
} __attribute__((packed)) uart_frame_t;  // 18字节

bool UART_HARDWARE_init(void);
void  uart_task(context_pack_t *ctx);
void uart_init(int baud,int UART_NUM,int TXD_PIN, int RXD_PIN,int RX_BUF_SIZE,int TX_BUF_SIZE);

int Uart_Send_Byte(int uart_num, uint8_t data);
int Uart_Send_Data(int uart_num, uint8_t* data, uint16_t len);
uint16_t Uart_Available(int uart_num);
uint8_t Uart_Read(int uart_num);
void Uart_Clean_Buffer(int uart_num);

#endif /* COMPONENTS_UART_INCLUDE_UART1_H_ */
