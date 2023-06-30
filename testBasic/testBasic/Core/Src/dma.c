/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
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

/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/* USER CODE BEGIN 2 */
void DMA_USART_Config( DMA_TypeDef * DMA_name,
                       uint32_t stream,
                       uint32_t channel,
                       uint32_t * buf,
                       uint32_t buflen,
                       USART_TypeDef * USARTx )
{
    LL_DMA_SetPeriphAddress( DMA_name, stream, ( uint32_t ) &( USARTx->RDR ) );/*设置外设地址，从哪里搬运数据 */
    LL_DMA_SetMemoryAddress( DMA_name, stream, ( uint32_t ) buf );            /*接收目标地址 */
    LL_DMA_SetDataLength( DMA_name, stream, buflen );                         /*设置DMA接受数据长度 */

    if( USARTx == USART2 )
    {
        LL_DMA_EnableIT_HT( DMA_name, stream );/*使能半满中断 */
        LL_DMA_EnableIT_TC( DMA_name, stream );/*使能DMA完成中断 */
    }

    if( USARTx == USART1 )
    {
              LL_DMA_ClearFlag_TC3(DMA_name);
      LL_DMA_ClearFlag_HT3(DMA_name);
      LL_DMA_EnableIT_TC( DMA_name, stream ); /*使能DMA完成中断 */
			LL_DMA_EnableIT_HT( DMA_name, stream ); /*使能DMA完成中断 */
    }

    if (USARTx==UART7)
    {
      LL_DMA_ClearFlag_TC3(DMA_name);
      LL_DMA_ClearFlag_HT3(DMA_name);
      LL_DMA_EnableIT_TC( DMA_name, stream ); /*使能DMA完成中断 */
			LL_DMA_EnableIT_HT( DMA_name, stream ); /*使能DMA完成中断 */
    }

    LL_DMA_EnableStream( DMA_name, stream );/*使能通道 */
    LL_USART_EnableDMAReq_RX( USARTx );       /*使能接收 */

    
}
/* USER CODE END 2 */

