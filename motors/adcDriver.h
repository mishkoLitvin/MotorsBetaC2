/*
 * adcDriver.h
 *
 *  Created on: 12 вер. 2016
 *      Author: Mishko Lytvyn
 */

#include "F2806x_Adc.h"
#include "systemDriver.h"
#include "F2806x_PieCtrl.h"


#ifndef ADCDRIVER_H_
#define ADCDRIVER_H_

#define ADC_usDELAY  1000L
#define ADC_BASE_ADDR 			(0x007100)
#define ADC_RESULT_BASE_ADDR 	(0x000B00)


ADC_REGS* adcRegs;
ADC_RESULT_REGS* adcResRegs;

void adcInit();
void ADC_config();
int adcRead(int ch);

void adcInit()
{
	adcRegs = (ADC_REGS*)ADC_BASE_ADDR;
	adcResRegs = (ADC_RESULT_REGS*) ADC_RESULT_BASE_ADDR;
}

void ADC_config()
{
	EALLOW;
	sysCtrlRegs->PCLKCR0.bit.ADCENCLK = 1;
	(*Device_cal)();
	EDIS;
	// Configure ADC
	EALLOW;
	adcRegs->ADCCTL1.bit.ADCBGPWD  = 1;      // Power ADC BG
	adcRegs->ADCCTL1.bit.ADCREFPWD = 1;      // Power reference
	adcRegs->ADCCTL1.bit.ADCPWDN   = 1;      // Power ADC
	adcRegs->ADCCTL1.bit.ADCENABLE = 1;      // Enable ADC
	adcRegs->ADCCTL1.bit.ADCREFSEL = 0;      // Select interal BG
	EDIS;

	int i;
	for(i=0; i<10000; i++){}
	//	DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels

	EALLOW;
	adcRegs->ADCCTL2.bit.CLKDIV2EN = 1;
	adcRegs->ADCCTL2.bit.CLKDIV4EN = 0;
	EDIS;

	EALLOW;
	adcRegs->ADCCTL2.bit.ADCNONOVERLAP = 1;	// Enable non-overlap mode
	adcRegs->ADCCTL1.bit.INTPULSEPOS= 1;	// ADCINT1 trips after AdcResults latch

//	adcRegs->INTSEL1N2.bit.INT1SEL   = 1;
//	adcRegs->INTSEL1N2.bit.INT1E     = 1;

	adcRegs->INTSEL1N2.bit.INT1SEL 	= 1;    // setup EOC1 to trigger ADCINT1 to fire
	adcRegs->ADCSOC0CTL.bit.CHSEL 	= 0;    // set SOC0 channel select to ADCINA0
	adcRegs->ADCSOC1CTL.bit.CHSEL 	= 1;    // set SOC1 channel select to ADCINA1
	adcRegs->ADCSOC2CTL.bit.CHSEL 	= 2;    // set SOC1 channel select to ADCINA1
	adcRegs->ADCSOC3CTL.bit.CHSEL 	= 3;    // set SOC1 channel select to ADCINA1
	adcRegs->ADCSOC4CTL.bit.CHSEL 	= 4;    // set SOC1 channel select to ADCINA1
	adcRegs->ADCSOC5CTL.bit.CHSEL 	= 5;    // set SOC1 channel select to ADCINA1
	adcRegs->ADCSOC6CTL.bit.CHSEL 	= 6;    // set SOC1 channel select to ADCINA1
	adcRegs->ADCSOC7CTL.bit.CHSEL 	= 7;    // set SOC1 channel select to ADCINA1
	adcRegs->ADCSOC8CTL.bit.CHSEL 	= 8;    // set SOC1 channel select to ADCINA1
	adcRegs->ADCSOC9CTL.bit.CHSEL 	= 9;    // set SOC1 channel select to ADCINA1
	adcRegs->ADCSOC10CTL.bit.CHSEL 	= 10;    // set SOC1 channel select to ADCINA1
	adcRegs->ADCSOC11CTL.bit.CHSEL 	= 11;    // set SOC1 channel select to ADCINA1
	adcRegs->ADCSOC12CTL.bit.CHSEL 	= 12;    // set SOC1 channel select to ADCINA1
	adcRegs->ADCSOC13CTL.bit.CHSEL 	= 13;    // set SOC1 channel select to ADCINA1
	adcRegs->ADCSOC14CTL.bit.CHSEL 	= 14;    // set SOC1 channel select to ADCINA1
	adcRegs->ADCSOC15CTL.bit.CHSEL 	= 15;    // set SOC1 channel select to ADCINA1

	adcRegs->ADCSOC0CTL.bit.TRIGSEL = 5;    // set SOC0 start trigger on timer0
	adcRegs->ADCSOC1CTL.bit.TRIGSEL = 5;    // set SOC1 start trigger on timer0
	adcRegs->ADCSOC2CTL.bit.TRIGSEL = 5;    // set SOC2 start trigger on timer0
	adcRegs->ADCSOC3CTL.bit.TRIGSEL = 5;    // set SOC3 start trigger on timer0
	adcRegs->ADCSOC4CTL.bit.TRIGSEL = 5;    // set SOC4 start trigger on timer0
	adcRegs->ADCSOC5CTL.bit.TRIGSEL = 1;    // set SOC5 start trigger on timer0
	adcRegs->ADCSOC6CTL.bit.TRIGSEL = 5;    // set SOC6 start trigger on timer0
	adcRegs->ADCSOC7CTL.bit.TRIGSEL = 1;    // set SOC7 start trigger on timer0
	adcRegs->ADCSOC8CTL.bit.TRIGSEL = 5;    // set SOC8 start trigger on timer0
	adcRegs->ADCSOC9CTL.bit.TRIGSEL = 5;    // set SOC9 start trigger on timer0
	adcRegs->ADCSOC10CTL.bit.TRIGSEL = 5;   // set SOC10 start trigger on timer0
	adcRegs->ADCSOC11CTL.bit.TRIGSEL = 5;   // set SOC11 start trigger on timer0
	adcRegs->ADCSOC12CTL.bit.TRIGSEL = 5;   // set SOC12 start trigger on timer0
	adcRegs->ADCSOC13CTL.bit.TRIGSEL = 1;   // set SOC13 start trigger on timer0
	adcRegs->ADCSOC14CTL.bit.TRIGSEL = 5;   // set SOC14 start trigger on timer0
	adcRegs->ADCSOC15CTL.bit.TRIGSEL = 1;   // set SOC15 start trigger on timer0

	adcRegs->ADCSOC0CTL.bit.ACQPS 	= 8;	// set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC1CTL.bit.ACQPS 	= 8;	// set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC2CTL.bit.ACQPS 	= 8;	// set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC3CTL.bit.ACQPS 	= 8;	// set SOC3 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC4CTL.bit.ACQPS 	= 8;	// set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC5CTL.bit.ACQPS 	= 8;	// set SOC5 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC6CTL.bit.ACQPS 	= 8;	// set SOC6 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC7CTL.bit.ACQPS 	= 8;	// set SOC7 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC8CTL.bit.ACQPS 	= 8;	// set SOC8 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC9CTL.bit.ACQPS 	= 8;	// set SOC9 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC10CTL.bit.ACQPS 	= 8;	// set SOC10 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC11CTL.bit.ACQPS 	= 8;	// set SOC11 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC12CTL.bit.ACQPS 	= 8;	// set SOC12 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC13CTL.bit.ACQPS 	= 8;	// set SOC13 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC14CTL.bit.ACQPS 	= 8;	// set SOC14 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	adcRegs->ADCSOC15CTL.bit.ACQPS 	= 8;	// set SOC15 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	EDIS;

}


int adcRead(int ch)
{
//	adcRegs->ADCSOCFRC1.all = (1<<ch); // Sample ADC input

	switch (ch) {
	case 0:
		return adcResRegs->ADCRESULT0;
	case 1:
		return adcResRegs->ADCRESULT1;
	case 2:
		return adcResRegs->ADCRESULT2;
	case 3:
		return adcResRegs->ADCRESULT3;
	case 4:
		return adcResRegs->ADCRESULT4;
	case 5:
		return adcResRegs->ADCRESULT5;
	case 6:
		return adcResRegs->ADCRESULT6;
	case 7:
		return adcResRegs->ADCRESULT7;
	case 8:
		return adcResRegs->ADCRESULT8;
	case 9:
		return adcResRegs->ADCRESULT9;
	case 10:
		return adcResRegs->ADCRESULT10;
	case 11:
		return adcResRegs->ADCRESULT11;
	case 12:
		return adcResRegs->ADCRESULT12;
	case 13:
		return adcResRegs->ADCRESULT13;
	case 14:
		return adcResRegs->ADCRESULT14;
	case 15:
		return adcResRegs->ADCRESULT15;
	default:
		break;
	}
	return 0;
}


#endif /* ADCDRIVER_H_ */
