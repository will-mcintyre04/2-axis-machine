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
TIM_HandleTypeDef htim2;

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

//#define DEBOUNCE_DELAY_MS 50 // Debounce delay in milliseconds

//volatile uint8_t debounce_active = 0;

void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 79; 
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 10000;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        while (1);
    }

    // Enable interrupt
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void Init_Input_Pin_GPIOB(uint32_t pin){
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Init_Input_Pin_GPIOA(uint32_t pin){
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
  GPIO_InitTypeDef GPIOA1_InitStruct;

  GPIOA_InitStruct.Pull = GPIO_PULLUP;
  GPIOA_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIOA_InitStruct.Speed = GPIO_SPEED_FAST;

  GPIOB_InitStruct.Pull = GPIO_PULLUP;
  GPIOB_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIOB_InitStruct.Speed = GPIO_SPEED_FAST;

  GPIOC_InitStruct.Pull = GPIO_PULLUP;
  GPIOC_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIOC_InitStruct.Speed = GPIO_SPEED_FAST;

  GPIOA1_InitStruct.Pull = GPIO_PULLUP;
  GPIOA1_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIOA1_InitStruct.Speed = GPIO_SPEED_FAST;

  GPIOA_InitStruct.Pin = GPIO_PIN_8;
  GPIOA1_InitStruct.Pin = GPIO_PIN_9;
  GPIOB_InitStruct.Pin = GPIO_PIN_6;
  GPIOC_InitStruct.Pin = GPIO_PIN_7;

  HAL_GPIO_Init(GPIOA, &GPIOA_InitStruct);
  HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);
  HAL_GPIO_Init(GPIOC, &GPIOC_InitStruct);
  HAL_GPIO_Init(GPIOA, &GPIOA1_InitStruct);
  
  /* Enable and set Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0x00, 0x00);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}


/**
  * @}
  */ /* End of ExampleTypes */

/**
  * @brief The FW main module
  */

int main(void)
{
  uint16_t raw;
  char msg[20];
  /* NUCLEO board initialization */
  NUCLEO_Board_Init();
  //MX_TIM2_Init();
  MX_ADC1_Init();
  
  /* X-NUCLEO-IHM02A1 initialization */
  BSP_Init();

  Init_Input_Pin_GPIOA(GPIO_PIN_1); //PA1 is ADC_in
  Init_Interrupt_Pin_GPIO_9_5();

  USART_Transmit(&huart2, "Initialized\n");
  
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

  //Start timer
  //HAL_TIM_Base_Start(&htim2);
  
  /* Infinite loop */

  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) ==GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_RESET){
    L6470_PrepareRun(0,1,1000);
    L6470_Run(0,1,1000);
  }
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET){
    L6470_PrepareRun(1,1,1000);
    L6470_Run(1,1,1000);
  }
  USART_Transmit(&huart2, "Motor starting\n\r");
  //HAL_ADC_Start(&hadc1); //enables continous conversion

  while (1)
  {
    uint32_t sum = 0;
    uint16_t raw;
    char msg[50];

    // Get ADC value
    for (int i = 0; i < 100; i++) {
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
      raw = HAL_ADC_GetValue(&hadc1);
      HAL_ADC_Stop(&hadc1);
      sum += raw;
    }

    // Calculate average with floating point
    uint32_t avg_int = sum / 100;
    uint32_t avg_frac = ((sum * 100) / 100) % 100;  // Get 2 decimal places

    // Convert to string and print
    sprintf(msg, "ADC Avg: %lu.%02lu\r\n", avg_int, avg_frac);
    USART_Transmit(&huart2, msg);
    HAL_Delay(1000);
    


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
