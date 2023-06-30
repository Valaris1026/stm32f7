/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "lwrb.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define BUF_MAXLEN 2048

typedef struct __USART_H__
{
  void (*USART_Start)(USART_TypeDef *USART);
  void (*USART_Stop)(USART_TypeDef *USART);
  void (*USART_Reset)(USART_TypeDef *USART);
  void (*USART_Clear)(USART_TypeDef *USART);
  void (*USART_Send)(USART_TypeDef *USART,uint8_t *buf, uint32_t size);
  uint16_t (*USART_CRC)(uint8_t *ptr,int len,uint16_t *CRC16Temp);
}USART_ACT_t;


typedef struct _USART_ELEMENT_
{
  uint32_t Rcv_len_Temp;/*Current received data lenth*/
  uint32_t Rcv_total_len;/*Total length of received data.Rcv_total_len = */
  uint16_t TC_count;/*Transfer completed counter.*/
  uint16_t HT_count;/*Transfer half completed counter.*/
  // bool Is_Half_Transfer;/*if 'true' means date transfer from dma to usart end with half transfer,otherwise end with full transfer*/
  bool Is_Msg_Rcv;/*if new message is received.*/
  lwrb_t lwrb_handle;/*lwrb handle*/
}USART_Token_t;
/* USER CODE END Private defines */

void MX_UART7_Init(void);
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void USART_FSM_Init(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

