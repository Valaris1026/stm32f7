/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern volatile USART_Token_t USART1_Token;
extern volatile USART_Token_t UART7_Token;
 
extern volatile uint8_t U7_RX_BUF[BUF_MAXLEN] ;
extern volatile uint8_t U1_RX_BUF[BUF_MAXLEN] ;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
      if( LL_DMA_IsActiveFlag_HT3( DMA1 ) ) /**/
    {
        LL_DMA_ClearFlag_HT3( DMA1 );
        // UART7_Token.Is_Msg_Rcv = true;
        // UART7_Token.Rcv_len_Temp = BUF_MAXLEN / 2;
        // UART7_Token.HT_count++;
        // lwrb_write( ( lwrb_t * ) &UART7_Token.lwrb_handle, ( uint8_t * ) U7_RX_BUF, ( uint32_t ) UART7_Token.Rcv_len_Temp ); /*Write in lwrb buffer*/
    }

    if( LL_DMA_IsActiveFlag_TC3( DMA1 ) ) /**/
    {
        // UART7_Token.Is_Msg_Rcv = true;
        LL_DMA_ClearFlag_TC3( DMA1 );
        // UART7_Token.TC_count++;
        // lwrb_write( ( lwrb_t * ) &UART7_Token.lwrb_handle, ( uint8_t * ) &U7_RX_BUF[ UART7_Token.Rcv_len_Temp ], ( uint32_t ) UART7_Token.Rcv_len_Temp ); /*Write in lwrb buffer*/
    }
  /* USER CODE END DMA1_Stream3_IRQn 0 */

  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
        if( LL_USART_IsActiveFlag_IDLE( USART1 ) )
    {
        LL_USART_ClearFlag_IDLE( USART1 );
        LL_DMA_DisableStream( DMA2, LL_DMA_STREAM_2 );

        USART1_Token.Is_Msg_Rcv = true;
        USART1_Token.Rcv_len_Temp = BUF_MAXLEN - (  LL_DMA_GetDataLength( DMA2, LL_DMA_STREAM_2 ) );

        // if( USART1_Token.HT_count != USART1_Token.TC_count )
        // {
        //     lwrb_write( ( lwrb_t * ) &USART1_Token.lwrb_handle, ( uint8_t * ) &U1_RX_BUF[ BUF_MAXLEN / 2 ], ( uint32_t ) ( USART1_Token.Rcv_len_Temp - BUF_MAXLEN / 2 ) );
        //     USART1_Token.Rcv_len_Temp = USART1_Token.Rcv_len_Temp + BUF_MAXLEN * USART1_Token.TC_count;
        // }
        // else
        // {
            lwrb_write( ( lwrb_t * ) &USART1_Token.lwrb_handle, ( uint8_t * ) U1_RX_BUF, ( uint32_t ) USART1_Token.Rcv_len_Temp );
            //addlist
            // USART1_Token.Rcv_len_Temp = USART1_Token.Rcv_len_Temp + BUF_MAXLEN * USART1_Token.TC_count;
        // }

        USART1_Token.HT_count = 0;
        USART1_Token.TC_count = 0;

        LL_DMA_SetDataLength( DMA2, LL_DMA_STREAM_2, BUF_MAXLEN );
        LL_DMA_EnableStream( DMA2, LL_DMA_STREAM_2 );
    }

    if( LL_USART_IsActiveFlag_ORE( USART1 ) ) /*Serial port overflow*/
    {
        LL_USART_ReceiveData8( USART1 );
        LL_USART_ClearFlag_ORE( USART1 );
    }
  
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */
  
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
      if( LL_DMA_IsActiveFlag_HT2( DMA2 ) ) /**/
    {
        LL_DMA_ClearFlag_HT2( DMA2 );
        // UART7_Token.Is_Msg_Rcv = true;
        // UART7_Token.Rcv_len_Temp = BUF_MAXLEN / 2;
        // UART7_Token.HT_count++;
        // lwrb_write( ( lwrb_t * ) &UART7_Token.lwrb_handle, ( uint8_t * ) U7_RX_BUF, ( uint32_t ) UART7_Token.Rcv_len_Temp ); /*Write in lwrb buffer*/
    }

    if( LL_DMA_IsActiveFlag_TC2( DMA2 ) ) /**/
    {
        // UART7_Token.Is_Msg_Rcv = true;
        LL_DMA_ClearFlag_TC2( DMA2 );
        // UART7_Token.TC_count++;
        // lwrb_write( ( lwrb_t * ) &UART7_Token.lwrb_handle, ( uint8_t * ) &U7_RX_BUF[ UART7_Token.Rcv_len_Temp ], ( uint32_t ) UART7_Token.Rcv_len_Temp ); /*Write in lwrb buffer*/
    }
  /* USER CODE END DMA2_Stream2_IRQn 0 */

  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles UART7 global interrupt.
  */
void UART7_IRQHandler(void)
{
  /* USER CODE BEGIN UART7_IRQn 0 */
      if( LL_USART_IsActiveFlag_IDLE( UART7 ) )
    {
        LL_USART_ClearFlag_IDLE( UART7 );
        LL_DMA_DisableStream( DMA1, LL_DMA_STREAM_3 );

        UART7_Token.Is_Msg_Rcv = true;
        UART7_Token.Rcv_len_Temp = BUF_MAXLEN - (  LL_DMA_GetDataLength( DMA1, LL_DMA_STREAM_3 ) );

        // if( UART7_Token.HT_count != UART7_Token.TC_count )
        // {
        //     lwrb_write( ( lwrb_t * ) &UART7_Token.lwrb_handle, ( uint8_t * ) &U7_RX_BUF[ BUF_MAXLEN / 2 ], ( uint32_t ) ( UART7_Token.Rcv_len_Temp - BUF_MAXLEN / 2 ) );
        //     UART7_Token.Rcv_len_Temp = UART7_Token.Rcv_len_Temp + BUF_MAXLEN * UART7_Token.TC_count;
        // }
        // else
        // {
            lwrb_write( ( lwrb_t * ) &UART7_Token.lwrb_handle, ( uint8_t * ) U7_RX_BUF, ( uint32_t ) UART7_Token.Rcv_len_Temp );
            //addlist
            // UART7_Token.Rcv_len_Temp = UART7_Token.Rcv_len_Temp + BUF_MAXLEN * UART7_Token.TC_count;
        // }

        UART7_Token.HT_count = 0;
        UART7_Token.TC_count = 0;

        LL_DMA_SetDataLength( DMA1, LL_DMA_STREAM_3, BUF_MAXLEN );
        LL_DMA_EnableStream( DMA1, LL_DMA_STREAM_3 );
    }

    if( LL_USART_IsActiveFlag_ORE( UART7 ) ) /*Serial port overflow*/
    {
        LL_USART_ReceiveData8( UART7 );
        LL_USART_ClearFlag_ORE( UART7 );
    }
  
  /* USER CODE END UART7_IRQn 0 */
  /* USER CODE BEGIN UART7_IRQn 1 */

  /* USER CODE END UART7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
