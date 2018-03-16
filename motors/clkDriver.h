/*
 * clkDriver.h
 *
 *  Created on: 12 вер. 2016
 *      Author: Mishko Lytvyn
 */

#include "clk.h"

#ifndef CLKDRIVER_H_
#define CLKDRIVER_H_

CLK_Handle clkS;

void clkInit();
void setupClks();
void setupPeripheralClks();


void clkInit()
{
	clkS = CLK_init((void *)CLK_BASE_ADDR,sizeof(CLK_Obj));
}

void setupClks()
{
	// enable internal oscillator 1
	CLK_enableOsc1(clkS);

	// disable the crystal oscillator
	CLK_disableCrystalOsc(clkS);

	// disable oscillator 2
	CLK_disableOsc2(clkS);

	// set the oscillator source pin GPIO19
	CLK_Obj *clkObj = (CLK_Obj *)clkS;
	ENABLE_PROTECTED_REGISTER_WRITE_MODE;
	clkObj->XCLK |= (1<<6);
	DISABLE_PROTECTED_REGISTER_WRITE_MODE;

	CLK_enableClkIn(clkS);
	CLK_setOscSrc(clkS,CLK_OscSrc_External);

	// set the low speed clock prescaler
	CLK_setLowSpdPreScaler(clkS,CLK_LowSpdPreScaler_SysClkOut_by_1);

	// set the clock out prescaler
	CLK_setClkOutPreScaler(clkS,CLK_ClkOutPreScaler_SysClkOut_by_1);

	return;
} // end of setupClks() function

void setupPeripheralClks()
{
	CLK_enableAdcClock(clkS);

	CLK_enableCompClock(clkS,CLK_CompNumber_1);
	CLK_enableCompClock(clkS,CLK_CompNumber_2);
	CLK_enableCompClock(clkS,CLK_CompNumber_3);

	CLK_enableEcap1Clock(clkS);

	CLK_disableEcanaClock(clkS);

	CLK_enablePwmClock(clkS,PWM_Number_1);
	CLK_enablePwmClock(clkS,PWM_Number_2);
	CLK_enablePwmClock(clkS,PWM_Number_3);
	CLK_enablePwmClock(clkS,PWM_Number_4);
	CLK_enablePwmClock(clkS,PWM_Number_5);
	CLK_enablePwmClock(clkS,PWM_Number_6);
	CLK_enablePwmClock(clkS,PWM_Number_7);


	CLK_disableHrPwmClock(clkS);

	CLK_disableI2cClock(clkS);

	CLK_disableLinAClock(clkS);

	CLK_disableClaClock(clkS);

	CLK_enableSciaClock(clkS);

	CLK_enableSpiaClock(clkS);
	CLK_disableSpibClock(clkS);

	CLK_enableTbClockSync(clkS);

	// McBSP clock enable
	CLK_Obj *clkObj = (CLK_Obj *)clkS;
	ENABLE_PROTECTED_REGISTER_WRITE_MODE;
	clkObj->PCLKCR0 |= (1<<12);
	DISABLE_PROTECTED_REGISTER_WRITE_MODE;

	CLK_enableCpuTimerClock(clkS, CLK_CpuTimerNumber_0);
	CLK_enableCpuTimerClock(clkS, CLK_CpuTimerNumber_1);
	CLK_enableCpuTimerClock(clkS, CLK_CpuTimerNumber_2);


	return;
} // end of setupPeripheralClks() function

#endif /* CLKDRIVER_H_ */
