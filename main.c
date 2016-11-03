#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/adc.h"

//#define FREQUENCY 10000;

#define MAX_TEMP 230

int main(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_OD);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);

	/*
	volatile uint16_t period, duty_cycle;
	volatile float adjust = .1;

	//pwm configuration -----------------------------------------------------------------
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PD1_M0PWM7);           //PD1 pin, PWM output 7
	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_1,GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_OD);

	period = SysCtlPWMClockGet()/FREQUENCY;	//PWM running at 625kHz
	duty_cycle = period*adjust;            	//adjust will vary at 10% duty cycle increses

	//configure period and duty cycle -----------------------------------------------------
	PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, duty_cycle);
	PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);

	// switch configuration --------------------------------------------------------------
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	*/

	//adc configuration----------------------------------------------------------------------
	uint32_t ui32ADC0Value[4];
	volatile uint32_t ui32TempAvg;
	volatile uint32_t ui32TempValueC;
	volatile uint32_t ui32TempValueF;

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Select the analog ADC function for these pins.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    //
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.  This example is arbitrarily using sequence 3.
    //
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // information on the ADC sequences and steps, reference the datasheet.
    //
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                             ADC_CTL_END);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    //
    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    ADCIntClear(ADC0_BASE, 3);

    //
    // Sample AIN0 forever.  Display the value on the console.

	while(1)
	{
		//adjust pwm according to button input, 1 push = +.1 duty cycle
		/*
		if( GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) == 0x00){
			adjust += .1;
			duty_cycle = period*adjust;
			if(adjust > 1){
				adjust = 0;
			}
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, duty_cycle);
		}*/

		//
		// Trigger the ADC conversion.
		//
		 ADCProcessorTrigger(ADC0_BASE, 3);

		 //
		 // Wait for conversion to be completed.
		 //
		 while(!ADCIntStatus(ADC0_BASE, 3, false))
		 {
		 }
		 //
		 // Clear the ADC interrupt flag.
		 //
		 ADCIntClear(ADC0_BASE, 3);
		 //
		 // Read ADC Value.
		 //
		 ADCSequenceDataGet(ADC0_BASE, 3, ui32ADC0Value);
		 ui32TempAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;
		 ui32TempValueC = (1475 - ((2475 * ui32TempAvg)) / 4096)/10;
		 ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;

		 if(ui32TempValueC > MAX_TEMP){
			 GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00);
		 }else{
			 GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
		 }

		 //SysCtlDelay(1000000);

	}
}
