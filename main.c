/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_pdl.h"		// because using PDL
#include "cy_systick.h"
/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
float R2Temp(float R);
uint32_t timeTick = 0;
void count_tick (){
	timeTick++;
}
uint32_t getTick(){return timeTick;}
/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*Interrupt function setup and vector-routine*/
#define GPIO_INTERRUPT_PRIORITY 		(7u)
#define PORT_INTR_MASK 					(0x00000001UL << CYBSP_USER_BTN_PORT_NUM)
cy_stc_sysint_t intrCfg;
uint32_t pressed_counter = 0;
uint8_t pressed_state = 0;
/* Interrupt callback function */
void GPIO_Interrupt_Handler(void){
	/* Get interrupt cause */
	uint32_t intrSrc = Cy_GPIO_GetInterruptCause0();
	/* Check if the interrupt was from the user button's port */
	if(PORT_INTR_MASK == (intrSrc & PORT_INTR_MASK)){
		/* Clear the interrupt */
		Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM);
		/* Place any additional interrupt code here */
		pressed_counter = ++pressed_counter>100?0:pressed_counter++;
		pressed_state = 1;
	}
}


void setup_interrupt(void)
{
	/* Interrupt config structure */
	intrCfg.intrSrc 		= CYBSP_USER_BTN_IRQ;
	intrCfg.intrPriority 	= GPIO_INTERRUPT_PRIORITY;
	/* Initialize the interrupt and register interrupt callback */
	 Cy_SysInt_Init(&intrCfg, &GPIO_Interrupt_Handler);
	 /* Enable the interrupt in the NVIC */
	 NVIC_EnableIRQ(intrCfg.intrSrc);
}
/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1.
*    2.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();
    // enabling vref
    Cy_SysAnalog_Init(&AREF_config);
    Cy_SysAnalog_Enable();
    // setup adc
    Cy_SAR_Init(HW_SAR_HW,&HW_SAR_config);
    Cy_SAR_Enable(HW_SAR_HW);



    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                     CY_RETARGET_IO_BAUDRATE);
    printf( "hallo \r\n");
    Cy_SysClk_ClkTimerDisable();
	Cy_SysClk_ClkTimerSetSource(CY_SYSCLK_CLKTIMER_IN_IMO);
	Cy_SysClk_ClkTimerSetDivider(7U);
	Cy_SysClk_ClkTimerEnable();

	Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_IMO, (8000000/1000)-1);
	Cy_SysTick_SetCallback(0,count_tick);
//    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_TIMER, 1000000U);

	//enabling PWM
	Cy_TCPWM_PWM_Init(digitalPWM_HW, digitalPWM_NUM, &digitalPWM_config);
	Cy_TCPWM_PWM_Enable(digitalPWM_HW, digitalPWM_NUM);
	Cy_TCPWM_TriggerReloadOrIndex(digitalPWM_HW, digitalPWM_MASK);


	// enabling interrupt pin
	setup_interrupt();

    int32_t value1, value2 = 0;
    uint32_t tickValue = 0;
    for (;;)
    {
    	if( getTick()- tickValue >= 1000 )
		{
    		tickValue = getTick();//Cy_TCPWM_Counter_GetCounter(my_tick_counter_HW, my_tick_counter_NUM);
    		Cy_GPIO_Clr(P10_3_PORT, P10_3_NUM);
			Cy_GPIO_Set(P10_0_PORT, P10_0_NUM);
			Cy_SysLib_Delay(10);
			// read reference
			Cy_SAR_StartConvert(HW_SAR_HW, CY_SAR_START_CONVERT_SINGLE_SHOT);
			if(Cy_SAR_IsEndConversion(HW_SAR_HW, CY_SAR_WAIT_FOR_RESULT) == CY_SAR_SUCCESS )
				value1 = Cy_SAR_GetResult16(SAR, 0);
			else
				value1 = 0;

			Cy_GPIO_Set(P10_3_PORT, P10_3_NUM);
			Cy_GPIO_Clr(P10_0_PORT, P10_0_NUM);
			Cy_SysLib_Delay(10);
			// read divided-voltage
			Cy_SAR_StartConvert(HW_SAR_HW, CY_SAR_START_CONVERT_SINGLE_SHOT);
			if(Cy_SAR_IsEndConversion(HW_SAR_HW, CY_SAR_WAIT_FOR_RESULT) == CY_SAR_SUCCESS )
				value2 = Cy_SAR_GetResult16(SAR, 0);
			else
				value2 = 0;
			Cy_GPIO_Clr(P10_3_PORT, P10_3_NUM);
			Cy_GPIO_Clr(P10_0_PORT, P10_0_NUM);

			float thermistor = ( (float)value2*10000.f ) / (float)value1;
			float temperature = R2Temp(thermistor);

			printf("ticking %u  : %u : %u \r\n", value1, value2, tickValue);
			printf("thermistor : %f  | %f%cC\r\n", thermistor, temperature, (char)176);

		}
    	if( pressed_state )
    	{
    		printf(" pressed = %d \r\n", pressed_counter);
    		if( pressed_counter %2 == 0)
    		{
//    			Cy_TCPWM_TriggerStopOrKill_Single(digitalPWM_HW, digitalPWM_NUM);
    			Cy_TCPWM_PWM_SetPeriod0(digitalPWM_HW, digitalPWM_NUM, 16384);
//				Cy_TCPWM_TriggerStart_Single(digitalPWM_HW, digitalPWM_NUM);
    		}
//    			Cy_GPIO_Clr(P13_7_PORT, P13_7_NUM);
    		else
    		{
//    			Cy_TCPWM_TriggerStopOrKill_Single(digitalPWM_HW, digitalPWM_NUM);
    			Cy_TCPWM_PWM_SetPeriod0(digitalPWM_HW, digitalPWM_NUM, 32767);
//    			Cy_TCPWM_PWM_SetCounter(digitalPWM_HW, digitalPWM_NUM, 100);
//				Cy_TCPWM_TriggerStart_Single(digitalPWM_HW, digitalPWM_NUM);
    		}
//    			Cy_GPIO_Set(P13_7_PORT, P13_7_NUM);
    		pressed_state =0;
    	}

    }
}


#define NTC_A 	1028444e-9
#define NTC_B 	239243e-9
#define NTC_C 	156e-9
#define R0 		10000.000f
float R2Temp(float R)
{
	float ret = 0;
	ret = log(R);
	float tempInC = (1/ (NTC_A + (NTC_B * ret) + (NTC_C * ret * ret * ret))) - 273.15;
	ret = tempInC;
	return ret;
}


/* [] END OF FILE */
