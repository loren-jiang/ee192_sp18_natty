/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*System includes.*/
#include <stdio.h>

#include "board.h"
#include "fsl_uart.h"

#include "pin_mux.h"
#include "clock_config.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_ftm.h"
#include "fsl_pit.h"  /* periodic interrupt timer */

#include "fsl_adc16.h"


//#include "telemetry/telemetry_uart.h"
//#include "telemetry/telemetry.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//PIT//
#define PIT_IRQ_ID PIT0_IRQn
/* Get source clock for PIT driver */
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

// Use pins D1 and C12 for SI and CLK on the camera
#define GPIO_CHANNEL_C GPIOC
#define GPIO_CHANNEL_D GPIOD
//#define GPIO_PIN_SI 3U
#define GPIO_PIN_SI 1U
#define GPIO_PIN_CLK 12U
#define GPIO_PIN_PWM 4U

/* The Flextimer instance/channel used for board. Indicates pin PTC2 (motor) and PTC1 (servo), see pin view for more info */
#define BOARD_FTM_BASEADDR0 FTM0
#define BOARD_FTM_BASEADDR3 FTM3
#define BOARD_FTM_CHANNEL0 kFTM_Chnl_0
#define BOARD_FTM_CHANNEL3 kFTM_Chnl_3
/* Interrupt to enable and flag to read; depends on the FTM channel used */
#define FTM_CHANNEL_INTERRUPT_ENABLE kFTM_Chnl0InterruptEnable
#define FTM_CHANNEL_FLAG kFTM_Chnl0Flag
#define FTM_CHANNEL_INTERRUPT_ENABLE kFTM_Chnl1InterruptEnable
#define FTM_CHANNEL_FLAG kFTM_Chnl1Flag


/* Interrupt number and interrupt handler for the FTM instance used */
#define FTM_INTERRUPT_NUMBER FTM0_IRQn
#define FTM_0_HANDLER FTM0_IRQHandler

/* Get source clock for FTM driver */
/* For slow PWM must pick a slow clock!!!!
 * kCLOCK_PlatClk (120 Mhz): freq > 914 hz
 * kCLOCK_BusClk (60 Mhz): freq > 457 hz
 * kCLOCK_FlexBusClk (40 Mhz): freq > 305 hz
 * kCLOCK_FlashClk (24 Mhz): freq > 183 hz
 * see fsl_clock.c for other clocks
 *
 * This example uses PWM @ 20khz so you can use any clock
 *
 */
/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK0 CLOCK_GetFreq(kCLOCK_BusClk)
#define FTM_SOURCE_CLOCK3 CLOCK_GetFreq(kCLOCK_BusClk)
//High Voltage(3.3V)=True for pwm
#define PWM_LEVEL kFTM_HighTrue

//ADC//s
#define DEMO_ADC16_BASEADDR ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 12U /* PTB2, ADC0_SE12 */
#define DEMO_ADC16_IRQn ADC0_IRQn
#define DEMO_ADC16_IRQ_HANDLER_FUNC ADC0_IRQHandler
#define VREF_BRD 3.300
#define SE_12BIT 4096.0
/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool ftmIsrFlag = false;
volatile uint8_t duty_cycle = 1U;

volatile int time;
volatile int prev_time;
uint32_t servo_freq = 200; //38*2.63 = 100 Hz...
//uint32_t servo_freq = 100;
uint32_t motor_freq = 2500;
float motorPWM = 30;
float centerPWM = 27;
//float centerPWM = 15;
uint32_t camera[128];
volatile uint32_t prev_arg_max; //previous arg max
volatile bool pitIsrFlag = false;
volatile uint32_t systime = 0; //systime updated very 100 us = 4 days ==> NEED OVERFLOW protection
volatile float error = 0.;
volatile float prev_error = 0.;
volatile float error_sum = 0.;
volatile float kp = 0.1;
volatile float kd = 0.2;
volatile float ki = 0.0;
volatile bool g_Adc16ConversionDoneFlag = false;
volatile uint32_t g_Adc16ConversionValue = 0;
adc16_channel_config_t g_adc16ChannelConfigStruct;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief delay a while.
 */
//void delay(void);
void update_duty_cycle(uint32_t freq_hz, uint8_t updated_duty_cycle);
void init_pwm_servo(uint32_t freq_hz, uint8_t init_duty_cycle);
void init_pwm_motor(uint32_t freq_hz, uint8_t init_duty_cycle);

/* Initialize ADC16 */
static void ADC_Init(void);
static void init_board(void);
void read_ADC(void);

/*******************************************************************************
 * Code
 ******************************************************************************/

void set_SI_high(void){
	GPIO_PinWrite(GPIO_CHANNEL_D, GPIO_PIN_SI, 1); // Set SI high
}
void set_SI_low(void){
	GPIO_PinWrite(GPIO_CHANNEL_D, GPIO_PIN_SI, 0); // Set SI high
}
void set_CLK_high(void){
	GPIO_PinWrite(GPIO_CHANNEL_C, GPIO_PIN_CLK, 1); // Set SI high
}
void set_CLK_low(void){
	GPIO_PinWrite(GPIO_CHANNEL_C, GPIO_PIN_CLK, 0); // Set SI high
}


void read_ADC(void){
	g_Adc16ConversionDoneFlag = false;
	ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP, &g_adc16ChannelConfigStruct);

	//Block until the ADC Conversion is finished
	 while (!g_Adc16ConversionDoneFlag)
	 {
	 }
}

void init_ADC(void)
{
	EnableIRQ(DEMO_ADC16_IRQn);
    adc16_config_t adc16ConfigStruct;

    /* Configure the ADC16. */
    /*
     * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
     * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
     * adc16ConfigStruct.enableAsynchronousClock = true;
     * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
     * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
     * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
     * adc16ConfigStruct.enableHighSpeed = false;
     * adc16ConfigStruct.enableLowPower = false;
     * adc16ConfigStruct.enableContinuousConversion = false;
     */
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
#if defined(BOARD_ADC_USE_ALT_VREF)
    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
    ADC16_Init(DEMO_ADC16_BASEADDR, &adc16ConfigStruct);

    /* Make sure the software trigger is used. */
    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASEADDR, false);
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASEADDR))
    {
        PRINTF("\r\nADC16_DoAutoCalibration() Done.");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    /* Prepare ADC channel setting */
    g_adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
    g_adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true;

#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    g_adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
}

void init_board(void){
	// Configure an output pin with initial value 0
	    gpio_pin_config_t gpio_config = {
	                kGPIO_DigitalOutput, 0,
	            };

	/* initialize GPIO Pin */
	GPIO_PinInit(GPIO_CHANNEL_D, GPIO_PIN_SI, &gpio_config); // initialize GPIOB pin PTD1
	GPIO_PinInit(GPIO_CHANNEL_C, GPIO_PIN_CLK, &gpio_config); // initialize GPIOB pin PTC12

	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
}

void init_PIT(void)
{
	// start periodic interrupt timer- should be in its own file
	pit_config_t pitConfig;
	PIT_GetDefaultConfig(&pitConfig);
	/* Init pit module */
	PIT_Init(PIT, &pitConfig);
	/* Set timer period for channel 0 */

	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(100U, PIT_SOURCE_CLOCK)); // 100 us timing
	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
	/* Enable at the NVIC */
	EnableIRQ(PIT_IRQ_ID);
	/* Start channel 0 */
	PRINTF("\r\nStarting channel No.0 ...");
	PIT_StartTimer(PIT, kPIT_Chnl_0);
}

/*Initialize the Flexible Timer Module to Produce PWM with init_duty_cycle at freq_hz*/
void init_pwm_servo(uint32_t freq_hz, uint8_t init_duty_cycle)
{
	duty_cycle = init_duty_cycle;
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;

    /* Configure ftm params with for pwm freq- freq_hz, duty cycle- init_duty_cycle */
    ftmParam.chnlNumber = BOARD_FTM_CHANNEL0;
    ftmParam.level = PWM_LEVEL;
    ftmParam.dutyCyclePercent = init_duty_cycle;
    ftmParam.firstEdgeDelayPercent = 0U;

    FTM_GetDefaultConfig(&ftmInfo);
    ftmInfo.prescale = kFTM_Prescale_Divide_128; //needed for low freq
    /* Initialize FTM module */
    FTM_Init(BOARD_FTM_BASEADDR0, &ftmInfo);

    FTM_SetupPwm(BOARD_FTM_BASEADDR0, &ftmParam, 1U, kFTM_CenterAlignedPwm, freq_hz, FTM_SOURCE_CLOCK0);

    /* Enable channel interrupt flag.*/
    //FTM_EnableInterrupts(BOARD_FTM_BASEADDR0, FTM_CHANNEL_INTERRUPT_ENABLE);

    /* Enable at the NVIC */
    //EnableIRQ(FTM_INTERRUPT_NUMBER);

    FTM_StartTimer(BOARD_FTM_BASEADDR0, kFTM_SystemClock);
}

void init_pwm_motor(uint32_t freq_hz, uint8_t init_duty_cycle)
{
	duty_cycle = init_duty_cycle;
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;

    /* Configure ftm params with for pwm freq- freq_hz, duty cycle- init_duty_cycle */
    ftmParam.chnlNumber = BOARD_FTM_CHANNEL3;
    ftmParam.level = PWM_LEVEL;
    ftmParam.dutyCyclePercent = init_duty_cycle;
    ftmParam.firstEdgeDelayPercent = 0U;

    FTM_GetDefaultConfig(&ftmInfo);
    ftmInfo.prescale = kFTM_Prescale_Divide_1; //needed for low freq
    /* Initialize FTM module */
    FTM_Init(BOARD_FTM_BASEADDR3, &ftmInfo);

    FTM_SetupPwm(BOARD_FTM_BASEADDR3, &ftmParam, 1U, kFTM_CenterAlignedPwm, freq_hz, FTM_SOURCE_CLOCK3);

    /* Enable channel interrupt flag.*/
    //FTM_EnableInterrupts(BOARD_FTM_BASEADDR0, FTM_CHANNEL_INTERRUPT_ENABLE);

    /* Enable at the NVIC */
    //EnableIRQ(FTM_INTERRUPT_NUMBER);

    FTM_StartTimer(BOARD_FTM_BASEADDR3, kFTM_SystemClock);
}

void update_duty_cycle_servo(float updated_duty_cycle, float centerPWM)
{
	float input;
	//float u = updated_duty_cycle - centerPWM;
	if (updated_duty_cycle < centerPWM * 0.33) { //hard limits
		input = (uint8_t) (centerPWM * 0.33);
	}
	else if (updated_duty_cycle > centerPWM * 1.67 ) {//hard limits
		input = (uint8_t) (centerPWM *1.67);
	}
	else {
		input = (uint8_t) updated_duty_cycle;
	}

	//FTM_DisableInterrupts(BOARD_FTM_BASEADDR0, FTM_CHANNEL_INTERRUPT_ENABLE);

	/* Disable channel output before updating the dutycycle */
	FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR0, BOARD_FTM_CHANNEL0, 0U);

	/* Update PWM duty cycle */
	FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR0, BOARD_FTM_CHANNEL0, kFTM_CenterAlignedPwm, input);

	/* Software trigger to update registers */
	FTM_SetSoftwareTrigger(BOARD_FTM_BASEADDR0, true);

	/* Start channel output with updated dutycycle */
	FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR0, BOARD_FTM_CHANNEL0, PWM_LEVEL);

	/* Delay to view the updated PWM dutycycle */
	//delay(); //Can be removed when using PWM for realtime applications

	/* Enable interrupt flag to update PWM dutycycle */
	//FTM_EnableInterrupts(BOARD_FTM_BASEADDR0, FTM_CHANNEL_INTERRUPT_ENABLE);
}

void update_duty_cycle_motor(float updated_duty_cycle)
{
	uint8_t input;
	if (updated_duty_cycle <0.0) {
		input = 0;
	}
	else if (updated_duty_cycle > 30.0) {
		input = 30;
	}
	else {
		input = (uint8_t) updated_duty_cycle;
	}

	//FTM_DisableInterrupts(BOARD_FTM_BASEADDR0, FTM_CHANNEL_INTERRUPT_ENABLE);

	/* Disable channel output before updating the dutycycle */
	FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR3, BOARD_FTM_CHANNEL3, 0U);

	/* Update PWM duty cycle */
	FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR3, BOARD_FTM_CHANNEL3, kFTM_CenterAlignedPwm, input);

	/* Software trigger to update registers */
	FTM_SetSoftwareTrigger(BOARD_FTM_BASEADDR3, true);

	/* Start channel output with updated dutycycle */
	FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR3, BOARD_FTM_CHANNEL3, PWM_LEVEL);

	/* Delay to view the updated PWM dutycycle */
	//delay(); //Can be removed when using PWM for realtime applications

	/* Enable interrupt flag to update PWM dutycycle */
	//FTM_EnableInterrupts(BOARD_FTM_BASEADDR0, FTM_CHANNEL_INTERRUPT_ENABLE);
}

//Function for delaying based on systime
void delay(uint32_t t)
{
	uint32_t endTime;
	endTime = t + systime;
	while(systime < endTime){
		continue;
	}
}

//Dummy function for delaying
void dummy_delay(int t)
{
	uint32_t i;
	for(i=0; i< t; i++)
	{
		asm("nop");
	}
}

//Get the current time
uint32_t get_curr_time(){
	return systime;
}

//function to fill up camera array with data
void take_pic(){
	//turn si high
	set_SI_high();
	dummy_delay(10000); //10000 iterations dummy delay
	//turn clk high
	set_CLK_high();
	dummy_delay(10000);
	//turn si low
	set_SI_low();

	uint16_t k = 0;
	for(k=0; k < 128; k++){
		set_CLK_high();
		dummy_delay(10000);
		read_ADC();
		//camera[k] = (int)(g_Adc16ConversionValue * (VREF_BRD / SE_12BIT)); //will add DELAY() LATER
		camera[k] = (int)(g_Adc16ConversionValue);
		set_CLK_low();
		dummy_delay(10000);
	}
	//PRINTF("%i \n",camera[63]);
}

int get_arg_max(uint32_t arr[]){
	uint32_t arg_max = 0;
	uint32_t threshold = 420;
//	uint32_t pixelCount = 0;
//	uint32_t lowPixel = 0;
//	uint32_t highPixel = 0;
	int i;
	for (i = 1; i < 128; ++i) {
//		  if (arr[i] > threshold) {
//			  if (i > lowPixel && lowPixel == 0) {
//				  lowPixel = i;
//			  }
//			  highPixel = i;
//			  pixelCount++;
//		  }
	      if (arr[arg_max] < arr[i]){
	    	  arg_max = i;
	      }
	   }
	if (arr[arg_max] < threshold) {
		arg_max  = prev_arg_max;
	}
	if (abs(arg_max - prev_arg_max) > 15) {
		arg_max = prev_arg_max;
	}
//	if (pixelCount > 80) {
//		arg_max = (lowPixel + highPixel) / 2;
//	}
	prev_arg_max = arg_max;
	return arg_max;
}

/*!
 * @brief Main function
 */
int main(void)
{
	time = 0;

	init_board();
	//init_uart();
	init_pwm_servo(servo_freq, centerPWM);
	init_pwm_motor(motor_freq, 0);
	init_ADC();
	init_PIT();
	delay(50000); //delay 5 seconds

//	update_duty_cycle_motor(14.0);


	/*Track the variables time, motor_pwm, and camera. Function signature is:
	* void register_telemetry_variable(char* data_type, char* internal_name, char* display_name, char* units, uint32_t* value_pointer, uint32_t num_elements, float lower_bound, float upper_bound)
	*
	* data_type = "uint", "int", or "float"
	* internal_name = internal reference name used for the python plotter (must have one variable with internal_name ='time')
	* display_name = string used to label the axis on the plot
	* units = string used to denote the units of the dependent variable
	* value_pointer = pointer to the variable you want to track. Make sure the variable is global or that you malloc space (not recommended) for it
	* num_elements = number of elements to track here (i.e. 1=just 1 number, 128=array of 128 elements)
	* lower_bound = float representing a lower bound on the data (used for setting plot bounds)
	* upper_bound = float representing an upper bound on the data (used for setting plot bounds)
	*/
//	register_telemetry_variable("uint", "time", "Time", "ms", (uint32_t*) &time,  1, 0,  0.0);
//	register_telemetry_variable("float", "motor", "Motor PWM", "Percent DC", (uint32_t*) &motor_pwm,  1, 0.0f,  0.5f);
//	register_telemetry_variable("uint", "linescan", "Linescan", "ADC", (uint32_t*) &camera,  128, 0.0f,  0.0f);
//
//	//Tell the plotter what variables to plot. Send this once before the main loop
//	transmit_header();

    while (1)
    {
    	//delay(3000);
    	time = get_curr_time();
    	take_pic();
    	delay(200);
    	take_pic();
		//Send a telemetry packet with the values of all the variables.
		//Be careful as this uses a blocking write- could mess with timing of other software.
		int argMax = get_arg_max(camera);
//    	PRINTF("arg max is %i\n", argMax);
		//CONTROL
		error = 63 - argMax; //should be a float
		uint32_t dt = time - prev_time;
		float error_d = (error - prev_error) / dt;
		error_sum += error;
		prev_time = time;
		prev_error = error;
		//kd = 0;
		float u = (kp*error + kd*error_d + ki*error_sum); //ki = 0
//		if (error < 0) {
//			update_duty_cycle_servo(centerPWM * .2, centerPWM);
//		}
//		else {
//			update_duty_cycle_servo(centerPWM * 1.8, centerPWM);
//		}
		update_duty_cycle_servo(centerPWM + u, centerPWM);
		//update_duty_cycle_servo(25, 25); //testing pwm
    	}

    }



void PIT0_IRQHandler(void)
{
	systime++; /* hopefully atomic operation */
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	pitIsrFlag = true;
    /* Clear interrupt flag.*/
}

void DEMO_ADC16_IRQ_HANDLER_FUNC(void)
{
    g_Adc16ConversionDoneFlag = true;
    /* Read conversion result to clear the conversion completed flag. */
    g_Adc16ConversionValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
