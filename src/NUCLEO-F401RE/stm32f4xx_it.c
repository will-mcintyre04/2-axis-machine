/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @date    13/05/2015 09:14:38
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "xnucleoihm02a1_interface.h"
#include "example_usart.h"

uint32_t lastDebounceTime = 0; // the last time a limit switch was toggled
uint32_t debounceDelay = 5000; // the debounce time; increase if the output flickers

// extern debounce_active;
// extern TIM_HandleTypeDef htim2;

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @addtogroup STM32F4XX_IT
  * @{
  */

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
  * @addtogroup STM32F4XX_IT_Exported_Functions
  * @{
  */

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI Line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/**
* @brief This function handles EXTI Line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
  USART_ITCharManager(&huart2);
}

// void EXTI4_IRQHandler(void)
// {
//   /* EXTI line interrupt detected */
//   if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET) 
//   {
//     __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);

//     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//   }
// }

void EXTI9_5_IRQHandler(void)
{
  // Debouncing
  const int delay = 200000;
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET) {
    for (volatile int i=0; i < delay; i++) {
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET){
        return;
      }
      USART_Transmit(&huart2, "PIN 6");
      L6470_PrepareHardStop(0);
      L6470_HardStop(0);
  }

  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET) {
    for (volatile int i=0; i < delay; i++) {
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_RESET){
        return;
      }
      USART_Transmit(&huart2, "PIN 7");
      L6470_PrepareHardStop(0);
      L6470_HardStop(0);
  }

  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET) {
    for (volatile int i=0; i < delay; i++) {
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET){
        return;
      }
      USART_Transmit(&huart2, "PIN 8");
      L6470_PrepareHardStop(1);
      L6470_HardStop(1);
  }

  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET) {
    for (volatile int i=0; i < delay; i++) {
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET){
        return;
      }
      USART_Transmit(&huart2, "PIN 9");
      L6470_PrepareHardStop(1);
      L6470_HardStop(1);
  }
}

/**
* @brief This function handles EXTI Line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

// void TIM2_IRQHandler(void)
// {
//   if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
//   {
//     if(__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) !=RESET)
//     {
//       __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
//       HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
//       debounce_active = 0;
//     }
//   }
// }

/**
  * @}
  */ /* End of STM32F4XX_IT_Exported_Functions */

/**
  * @}
  */ /* End of STM32F4XX_IT */

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
