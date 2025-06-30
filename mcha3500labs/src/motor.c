#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "motor.h"

static TIM_HandleTypeDef _htim3;
static TIM_OC_InitTypeDef _sConfigPWM;

void motor_PWM_init(void)
{
    /* Enable TIM3 clock */
    /* Enable GPIOA clock */
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Initialise PA6 with:
        - Pin 6
        - Alternate function push-pull mode
        - No pull
        - High frequency
        - Alternate function 2 - Timer 3*/
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.Pin = GPIO_PIN_6;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Initialise timer 3 with:
        - Instance TIM3
        - Prescaler of 1
        - Counter mode up
        - Timer period to generate a 10kHz signal
        - Clock division of 0 */
    _htim3.Instance = TIM3;
    _htim3.Init.Prescaler = 1;
    _htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    // _htim3.Init.Period = TIMERPERIOD;
    _htim3.Init.Period = 99;
    _htim3.Init.ClockDivision = 0;

    /* Configure timer 3, channel 1 with:
        - Output compare mode PWM1
        - Pulse = 0
        - OC polarity high
        - Fast mode disabled */
    _sConfigPWM.OCMode = TIM_OCMODE_PWM1;
    // _sConfigPWM.Pulse = 0;
    _sConfigPWM.Pulse = (uint32_t)(99 * 0.25);
    _sConfigPWM.OCPolarity = TIM_OCPOLARITY_HIGH;
    _sConfigPWM.OCFastMode = TIM_OCFAST_DISABLE;

    /* Set initial Timer 3, channel 1 compare value */
    HAL_TIM_PWM_Init(&_htim3);
    HAL_TIM_PWM_ConfigChannel(&_htim3, &_sConfigPWM, TIM_CHANNEL_1);
    /* Start Timer 3, channel 1 */
    HAL_TIM_PWM_Start(&_htim3, TIM_CHANNEL_1);
}
