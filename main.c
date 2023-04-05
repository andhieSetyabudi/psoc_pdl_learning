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
#include "stdbool.h"
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



/*****************************************/
/* 				PDM microphone			 */
/*****************************************/
/* Define how many samples in a frame */
#define FRAME_SIZE                  (1024)
/* Noise threshold hysteresis */
#define THRESHOLD_HYSTERESIS        3u
/* Volume ratio for noise and print purposes */
#define VOLUME_RATIO                (4*FRAME_SIZE)
/* Desired sample rate. Typical values: 8/16/22.05/32/44.1/48kHz */
#define SAMPLE_RATE_HZ              8000u
/* Decimation Rate of the PDM/PCM block. Typical value is 64 */
#define DECIMATION_RATE             64u
/* Audio Subsystem Clock. Typical values depends on the desire sample rate:
- 8/16/48kHz    : 24.576 MHz
- 22.05/44.1kHz : 22.579 MHz */
#define AUDIO_SYS_CLOCK_HZ          24576000u
/* PDM/PCM Pins */
#define PDM_DATA                    P10_5
#define PDM_CLK                     P10_4

void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event);
void pdm_clock_init(void);

/* Interrupt flags */
volatile bool button_flag = false;
volatile bool pdm_pcm_flag = true;

/* Volume variables */
uint32_t volume = 0;
uint32_t noise_threshold = THRESHOLD_HYSTERESIS;

/* HAL Object */
cyhal_pdm_pcm_t pdm_pcm;
cyhal_clock_t   audio_clock;
cyhal_clock_t   pll_clock;

/* HAL Config */
const cyhal_pdm_pcm_cfg_t pdm_pcm_cfg =
{
    .sample_rate     = SAMPLE_RATE_HZ,
    .decimation_rate = DECIMATION_RATE,
    .mode            = CYHAL_PDM_PCM_MODE_STEREO,
    .word_length     = 16,  /* bits */
    .left_gain       = 0,   /* dB */
    .right_gain      = 0,   /* dB */
};

int16_t  audio_frame[FRAME_SIZE] = {0};

void setup_PDM_PDL(void);
void loop_PDM_PDL(void);
void PCM_ISR_Handler(void);
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
    // enabling PDM microphone
    setup_PDM_PDL();

    // enabling vref
    Cy_SysAnalog_Init(&AREF_config);
    Cy_SysAnalog_Enable();
    // setup adc
    Cy_SAR_Init(HW_SAR_HW,&HW_SAR_config);
    Cy_SAR_Enable(HW_SAR_HW);



    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                     CY_RETARGET_IO_BAUDRATE);
    printf("\x1b[2J\x1b[;H");
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

//			pdm_loop();
		}
    	loop_PDM_PDL();

    	if( pressed_state )
    	{
    		printf(" pressed = %d \r\n", pressed_counter);
   /*
    * To set duty cycle:
    * - Need to set the period based on frequency of pwm
    * 	 period = (tcpwm_clock_hz + (frequencyOfPWM_hz >> 1)) / frequencyOfPWM_hz;
    * - Need to set the compare
    * 	 compare = (uint32_t)(duty_cycle * 0.01f * period);
    * */
    		// get the TCPWM_Clock in HZ
//    		uint32_t tcpwm_clk = Cy_TCPWM_
    		if( pressed_counter %2 == 0)
    		{
//    			Cy_TCPWM_TriggerStopOrKill_Single(digitalPWM_HW, digitalPWM_NUM);
//    			Cy_TCPWM_PWM_SetPeriod0(digitalPWM_HW, digitalPWM_NUM, 16384);
    			Cy_TCPWM_PWM_SetCompare0(digitalPWM_HW, digitalPWM_NUM, 8192);
    			Cy_TCPWM_PWM_SetCompare1(digitalPWM_HW, digitalPWM_NUM, 8192);
    			Cy_TCPWM_TriggerCaptureOrSwap_Single(digitalPWM_HW, digitalPWM_NUM);
//    			Cy_TCPWM_PWM_SetCounter(digitalPWM_HW, digitalPWM_NUM, 0);
//				Cy_TCPWM_TriggerStart_Single(digitalPWM_HW, digitalPWM_NUM);
    		}
    		else
    		{
//    			Cy_TCPWM_TriggerStopOrKill_Single(digitalPWM_HW, digitalPWM_NUM);
//    			Cy_TCPWM_PWM_SetPeriod0(digitalPWM_HW, digitalPWM_NUM, 32767);
    			Cy_TCPWM_PWM_SetCompare0(digitalPWM_HW, digitalPWM_NUM, 1000);
    			Cy_TCPWM_PWM_SetCompare1(digitalPWM_HW, digitalPWM_NUM, 1000);
    			Cy_TCPWM_TriggerCaptureOrSwap_Single(digitalPWM_HW, digitalPWM_NUM);
//    			Cy_TCPWM_PWM_SetCounter(digitalPWM_HW, digitalPWM_NUM, 0);
//				Cy_TCPWM_TriggerStart_Single(digitalPWM_HW, digitalPWM_NUM);
    		}
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


/*******************************************************************************
* Function Name: pdm_pcm_isr_handler
********************************************************************************
* Summary:
*  PDM/PCM ISR handler. Set a flag to be processed in the main loop.
*
* Parameters:
*  arg: not used
*  event: event that occurred
*
*******************************************************************************/
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event)
{
    (void) arg;
    (void) event;

    pdm_pcm_flag = true;
}

/*******************************************************************************
* Function Name: clock_init
********************************************************************************
* Summary:
*  Initialize the clocks in the system.
*
*******************************************************************************/
void pdm_clock_init(void)
{
    /* Initialize the PLL */
    cyhal_clock_reserve(&pll_clock, &CYHAL_CLOCK_PLL[0]);
    cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
    cyhal_clock_set_enabled(&pll_clock, true, true);

    /* Initialize the audio subsystem clock (CLK_HF[1])
     * The CLK_HF[1] is the root clock for the I2S and PDM/PCM blocks */
    cyhal_clock_reserve(&audio_clock, &CYHAL_CLOCK_HF[1]);

    /* Source the audio subsystem clock from PLL */
    cyhal_clock_set_source(&audio_clock, &pll_clock);
    cyhal_clock_set_enabled(&audio_clock, true, true);
}

void pdm_init()
{
	pdm_clock_init();
	/* Initialize the PDM/PCM block */
	cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
	cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, NULL);
	cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
	cyhal_pdm_pcm_start(&pdm_pcm);
}

void pdm_loop()
{
	/* Check if any microphone has data to process */
	if (pdm_pcm_flag)
	{
		/* Clear the PDM/PCM flag */
		pdm_pcm_flag = 0;

		/* Reset the volume */
		volume = 0;

		/* Calculate the volume by summing the absolute value of all the
		 * audio data from a frame */
		for (uint32_t index = 0; index < FRAME_SIZE; index++)
		{
			volume += abs(audio_frame[index]);
			printf("data %d : %i \r\n", index+1, audio_frame[index]);
		}

		/* Prepare line to report the volume */
		printf("volume : %d \n\r", volume);
		/* Report the volume */
		for (uint32_t index = 0; index < (volume/VOLUME_RATIO); index++)
		{
			printf("-");
		}
		/* Setup to read the next frame */
		cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, FRAME_SIZE);
	}
}



/**
 *  PDM for microphone
 *
 */

cy_stc_sysint_t pdm_int_cfg;
bool PCM_flag = 0;

bool get_PCM_flag(void)
{
	return PCM_flag;
}

void PCM_ISR_Handler(void)
{
    /* Set the PCM flag */
	PCM_flag = 1;

    /* Disable PCM ISR to avoid multiple calls to this ISR */
    NVIC_DisableIRQ(pdm_int_cfg.intrSrc);
}

void setup_PDM_PDL(void)
{
	// need to setup interrupt first
	pdm_int_cfg.intrSrc 		= PDM_MICROPHONE_IRQ;
	pdm_int_cfg.intrPriority 	= 6u;

	Cy_SysInt_Init(&pdm_int_cfg, PCM_ISR_Handler);
	NVIC_EnableIRQ(pdm_int_cfg.intrSrc);

	Cy_PDM_PCM_Init(PDM_MICROPHONE_HW, &PDM_MICROPHONE_config);
	Cy_PDM_PCM_Enable(PDM_MICROPHONE_HW);
	Cy_PDM_PCM_ClearFifo(PDM_MICROPHONE_HW);
}


#define UART_STRING_SIZE            32u
#define SAMPLE_SIZE                 (4*128)
int32_t audioSamples[128];
/* Variable to store the volume captured by the microphone */
volatile uint32_t volume_;

/* Sum variable to calculate the volume */
uint32_t sumVolume = 0u;

/* Number of samples processed */
uint32_t numSamples = 0u;

/* Noise threshold */
uint32_t noiseThreshold = 0u;

/* Buffer to store the string to be sent over UART */
void loop_PDM_PDL(void)
{
	/* Check PCM flag to process data */
	if (PCM_flag)
	{
		/* Clear the PCM flag */
		PCM_flag = 0;

		/* Calculate the volume by summing the absolute value of all the data
		 * stored in the PCM FIFO */
		uint32_t numFifo = 128;
		for (uint8_t id = 0; id < numFifo; id++)
		{
			audioSamples[id] = (int32_t) Cy_PDM_PCM_ReadFifo(PDM_MICROPHONE_HW) ;
			sumVolume += abs(audioSamples[id]);
		}
		for (uint8_t id = 0; id < numFifo; id++)
		{
			printf("%i\r\n", audioSamples[id]/60);
		}
		/* Add to the number of samples */
		numSamples += numFifo;

		/* Clear the PCM interrupt */
		Cy_PDM_PCM_ClearInterrupt(PDM_MICROPHONE_HW, PDM_INTR_RX_TRIGGER_Msk);

		/* Re-enable PCM ISR */
		NVIC_EnableIRQ(pdm_int_cfg.intrSrc);

		/* Check if reached the sample size */
		if (numSamples >= SAMPLE_SIZE)
		{
			/* Calculate the volume based on sample size */
			volume = sumVolume/(SAMPLE_SIZE);

			/* Reset the number of samples and the volume sum */
			numSamples = 0;
			sumVolume = 0;

			/* Prepare line to report the volvume */
//			printf("\n\r");

			/* Report the volume over UART */
//			for (uint8_t id = 0; id < volume;  id++)
//			{
//				printf('-');
//			}
//			printf("\r\n");
		};
	};
}
/* [] END OF FILE */
