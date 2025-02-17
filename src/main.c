/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 09/10/2014 11:13:03
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
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

//test git

 
#include "example.h"
#include "example_usart.h"
#include "params.h"
#include "xnucleoihm02a1.h"

/**
  * @defgroup   MotionControl
  * @{
  */

/**
  * @addtogroup BSP
  * @{
  */

/**
  * @}
  */ /* End of BSP */

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @defgroup   ExampleTypes
  * @{
  */

//#define MICROSTEPPING_MOTOR_EXAMPLE        //!< Uncomment to performe the standalone example
#define MICROSTEPPING_MOTOR_USART_EXAMPLE  //!< Uncomment to performe the USART example
#if ((defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option only!"
#elif ((!defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (!defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option!"
#endif
#if (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined (NUCLEO_USE_USART)))
  #error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
#endif

#define DEBOUNCE_DELAY_MS 50 // Debounce delay in milliseconds

volatile uint8_t debounce_active = 0;

void Init_Input_Pin_GPIOB(uint32_t pin){
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Init_Output_Pin_GPIOA(uint32_t pin){
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Init_Interrupt_Pin_GPIO_9_5(){
  GPIO_InitTypeDef GPIOA_InitStruct;
  GPIO_InitTypeDef GPIOB_InitStruct;
  GPIO_InitTypeDef GPIOC_InitStruct;

  GPIOA_InitStruct.Pull = GPIO_PULLDOWN;
  GPIOA_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIOA_InitStruct.Speed = GPIO_SPEED_FAST;

  GPIOB_InitStruct.Pull = GPIO_PULLDOWN;
  GPIOB_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIOB_InitStruct.Speed = GPIO_SPEED_FAST;

  GPIOC_InitStruct.Pull = GPIO_PULLDOWN;
  GPIOC_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIOC_InitStruct.Speed = GPIO_SPEED_FAST;

  GPIOA_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIOB_InitStruct.Pin = GPIO_PIN_6;
  GPIOC_InitStruct.Pin = GPIO_PIN_7;

  HAL_GPIO_Init(GPIOA, &GPIOA_InitStruct);
  HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);
  HAL_GPIO_Init(GPIOC, &GPIOC_InitStruct);
  
  /* Enable and set Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0x0F, 0x00);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

void StopMotor(uint8_t board, uint8_t device){
  StepperMotorBoardHandle_t *StepperMotorBoardHandle;
  StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(board);

  StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardStop(device);

  // Execute the stop command
  StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
}

void RunMotor(uint8_t board, uint8_t device, uint32_t speed, uint32_t step)
{
    StepperMotorBoardHandle_t *StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(board);

    // Prepare and execute the move command
    StepperMotorBoardHandle->Command->Move(board, device, L6470_DIR_FWD_ID, step);
    while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board, device, BUSY_ID) == 0);
}


/**
  * @}
  */ /* End of ExampleTypes */

/**
  * @brief The FW main module
  */
int main(void)
{
  /* NUCLEO board initialization */
  NUCLEO_Board_Init();
  
  /* X-NUCLEO-IHM02A1 initialization */
  BSP_Init();

  //Init_Input_Pin_GPIOB(GPIO_PIN_0);
  Init_Output_Pin_GPIOA(GPIO_PIN_0);
  Init_Interrupt_Pin_GPIO_9_5();
  
#ifdef NUCLEO_USE_USART
  /* Transmit the initial message to the PC via UART */
  USART_TxWelcomeMessage();
  	USART_Transmit(&huart2, " X-CUBE-SPN2 v1.0.0\n\r");
#endif
  
#if defined (MICROSTEPPING_MOTOR_EXAMPLE)
  /* Perform a batch commands for X-NUCLEO-IHM02A1 */
  MicrosteppingMotor_Example_01();
  
  /* Infinite loop */
  while (1);
#elif defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)
  /* Fill the L6470_DaisyChainMnemonic structure */
  Fill_L6470_DaisyChainMnemonic();
	
	/*Initialize the motor parameters */
	Motor_Param_Reg_Init();
  
  /* Infinite loop */
  while (1)
  {

    RunMotor(0, 0, 5000, 5000);

    /* Check if any Application Command for L6470 has been entered by USART */
    //USART_CheckAppCmd();
 
    //USART_Transmit(&huart2,  ("PIN 5", HALFBYTE_F));
    
    // while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {}

    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

    // while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {}

    /*
    */ 

  }
#endif
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/**
  * @}
  */ /* End of MotionControl */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
