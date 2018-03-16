/*
 * clockDriver.h
 *
 *  Created on: 12 вер. 2016
 *      Author: Mishko Lytvyn
 */

#include "F2806x_SysCtrl.h"

#ifndef SYSTEMDRIVER_H_
#define SYSTEMDRIVER_H_

#define SYS_CTRL_REGS_BASE_ADDR (0x007010)

SYS_CTRL_REGS* sysCtrlRegs;

void systemInit();
void systemSetup();
void ServiceDog();
void DisableDog();
void InitPll(Uint16 val, Uint16 divsel);
void setUpPeripheralClocks();
void setIntOsc1();
void setIntOsc2();
void setXTALOsc();
void setExtOsc();



void systemInit()
{
	sysCtrlRegs = (SYS_CTRL_REGS*) SYS_CTRL_REGS_BASE_ADDR;
}

void systemSetup()
{
	DisableDog();
	EALLOW;
	sysCtrlRegs->PCLKCR0.bit.ADCENCLK = 1; // Enable ADC peripheral clock
	(*Device_cal)();
	sysCtrlRegs->PCLKCR0.bit.ADCENCLK = 0; // Return ADC clock to original state
	EDIS;
	setExtOsc();
	InitPll(18,0);
	setUpPeripheralClocks();
}

void ServiceDog()
{
	EALLOW;
	sysCtrlRegs->WDKEY = 0x0055;
	sysCtrlRegs->WDKEY = 0x00AA;
	EDIS;
}

void DisableDog()
{
	EALLOW;
	sysCtrlRegs->WDCR= 0x0068;
	EDIS;
}

void InitPll(Uint16 val, Uint16 divsel)
{
	volatile Uint16 iVol;

	// Make sure the PLL is not running in limp mode
	if (sysCtrlRegs->PLLSTS.bit.MCLKSTS != 0)
	{
		EALLOW;
		// OSCCLKSRC1 failure detected. PLL running in limp mode.
		// Re-enable missing clock logic.
		sysCtrlRegs->PLLSTS.bit.MCLKCLR = 1;
		EDIS;
		// Replace this line with a call to an appropriate
		// SystemShutdown(); function.
		__asm("        ESTOP0");     // Uncomment for debugging purposes
	}

	// DIVSEL MUST be 0 before PLLCR can be changed from
	// 0x0000. It is set to 0 by an external reset XRSn
	// This puts us in 1/4
	if (sysCtrlRegs->PLLSTS.bit.DIVSEL != 0)
	{
		EALLOW;
		sysCtrlRegs->PLLSTS.bit.DIVSEL = 0;
		EDIS;
	}

	// Change the PLLCR
	if (sysCtrlRegs->PLLCR.bit.DIV != val)
	{

		EALLOW;
		// Before setting PLLCR turn off missing clock detect logic
		sysCtrlRegs->PLLSTS.bit.MCLKOFF = 1;
		sysCtrlRegs->PLLCR.bit.DIV = val;
		EDIS;
		DisableDog();

		while(sysCtrlRegs->PLLSTS.bit.PLLLOCKS != 1)
		{
			// Uncomment to service the watchdog
			// ServiceDog();
		}

		EALLOW;
		sysCtrlRegs->PLLSTS.bit.MCLKOFF = 0;
		EDIS;
	}

	// If switching to 1/2
	if((divsel == 1)||(divsel == 2))
	{
		EALLOW;
		sysCtrlRegs->PLLSTS.bit.DIVSEL = divsel;
		EDIS;
	}

	// If switching to 1/1
	if(divsel == 3)
	{
		EALLOW;
		sysCtrlRegs->PLLSTS.bit.DIVSEL = 2;
		usDelay(50L);
		sysCtrlRegs->PLLSTS.bit.DIVSEL = 3;
		EDIS;
	}
}


void setUpPeripheralClocks()
{
   EALLOW;

   sysCtrlRegs->LOSPCP.all = 0x0000;

// XCLKOUT to SYSCLKOUT ratio.  By default XCLKOUT = 1/1 SYSCLKOUT
   sysCtrlRegs->XCLK.bit.XCLKOUTDIV=2;

// Peripheral clock enables set for the selected peripherals.
// If you are not using a peripheral leave the clock off
// to save on power.
//
// Note: not all peripherals are available on all F2806x derivates.
// Refer to the datasheet for your particular device.
//
// This function is not written to be an example of efficient code.

   sysCtrlRegs->PCLKCR1.bit.EPWM1ENCLK = 1;    // ePWM1
   sysCtrlRegs->PCLKCR1.bit.EPWM2ENCLK = 1;    // ePWM2
   sysCtrlRegs->PCLKCR1.bit.EPWM3ENCLK = 1;    // ePWM3
   sysCtrlRegs->PCLKCR1.bit.EPWM4ENCLK = 1;    // ePWM4
   sysCtrlRegs->PCLKCR1.bit.EPWM5ENCLK = 1;    // ePWM5
   sysCtrlRegs->PCLKCR1.bit.EPWM6ENCLK = 1;    // ePWM6
   sysCtrlRegs->PCLKCR1.bit.EPWM7ENCLK = 0;    // ePWM7
   sysCtrlRegs->PCLKCR1.bit.EPWM8ENCLK = 0;    // ePWM8

   sysCtrlRegs->PCLKCR0.bit.HRPWMENCLK = 1;    // HRPWM
   sysCtrlRegs->PCLKCR0.bit.TBCLKSYNC = 1;     // Enable TBCLK within the ePWM

   sysCtrlRegs->PCLKCR1.bit.EQEP1ENCLK = 0;    // eQEP1
   sysCtrlRegs->PCLKCR1.bit.EQEP2ENCLK = 0;    // eQEP2

   sysCtrlRegs->PCLKCR1.bit.ECAP1ENCLK = 0;    // eCAP1
   sysCtrlRegs->PCLKCR1.bit.ECAP2ENCLK = 0;    // eCAP2
   sysCtrlRegs->PCLKCR1.bit.ECAP3ENCLK = 0;    // eCAP3

   sysCtrlRegs->PCLKCR2.bit.HRCAP1ENCLK = 0;	  // HRCAP1
   sysCtrlRegs->PCLKCR2.bit.HRCAP2ENCLK = 0;	  // HRCAP2
   sysCtrlRegs->PCLKCR2.bit.HRCAP3ENCLK = 0;	  // HRCAP3
   sysCtrlRegs->PCLKCR2.bit.HRCAP4ENCLK = 0;   // HRCAP4

   sysCtrlRegs->PCLKCR0.bit.ADCENCLK = 1;      // ADC
   sysCtrlRegs->PCLKCR3.bit.COMP1ENCLK = 1;    // COMP1
   sysCtrlRegs->PCLKCR3.bit.COMP2ENCLK = 1;    // COMP2
   sysCtrlRegs->PCLKCR3.bit.COMP3ENCLK = 1;    // COMP3

   sysCtrlRegs->PCLKCR3.bit.CPUTIMER0ENCLK = 1; // CPU Timer 0
   sysCtrlRegs->PCLKCR3.bit.CPUTIMER1ENCLK = 1; // CPU Timer 1
   sysCtrlRegs->PCLKCR3.bit.CPUTIMER2ENCLK = 1; // CPU Timer 2

   sysCtrlRegs->PCLKCR3.bit.DMAENCLK = 1;      // DMA

   sysCtrlRegs->PCLKCR3.bit.CLA1ENCLK = 1;     // CLA1

   sysCtrlRegs->PCLKCR3.bit.USB0ENCLK = 0;	  // USB0

   sysCtrlRegs->PCLKCR0.bit.I2CAENCLK = 0;     // I2C-A
   sysCtrlRegs->PCLKCR0.bit.SPIAENCLK = 1;     // SPI-A
   sysCtrlRegs->PCLKCR0.bit.SPIBENCLK = 0;     // SPI-B
   sysCtrlRegs->PCLKCR0.bit.SCIAENCLK = 1;     // SCI-A
   sysCtrlRegs->PCLKCR0.bit.SCIBENCLK = 0;     // SCI-B
   sysCtrlRegs->PCLKCR0.bit.MCBSPAENCLK = 1;   // McBSP-A
   sysCtrlRegs->PCLKCR0.bit.ECANAENCLK=0;      // eCAN-A

   sysCtrlRegs->PCLKCR0.bit.TBCLKSYNC = 1;     // Enable TBCLK within the ePWM

   EDIS;
}

// This function switches to Internal Oscillator 1 and turns off all other clock
// sources to minimize power consumption

void setIntOsc1(void)
{
    EALLOW;
    sysCtrlRegs->CLKCTL.bit.INTOSC1OFF = 0;
    sysCtrlRegs->CLKCTL.bit.OSCCLKSRCSEL=0;  // Clk Src = INTOSC1
    sysCtrlRegs->CLKCTL.bit.XCLKINOFF=1;     // Turn off XCLKIN
    sysCtrlRegs->CLKCTL.bit.XTALOSCOFF=1;    // Turn off XTALOSC
    sysCtrlRegs->CLKCTL.bit.INTOSC2OFF=1;    // Turn off INTOSC2
    EDIS;
}

// NOTE: If there is no external clock connection, when switching from
//       INTOSC1 to INTOSC2, EXTOSC and XLCKIN must be turned OFF prior
//       to switching to internal oscillator 1

void setIntOsc2 (void)
{
    EALLOW;
    sysCtrlRegs->CLKCTL.bit.INTOSC2OFF = 0;     // Turn on INTOSC2
    sysCtrlRegs->CLKCTL.bit.OSCCLKSRC2SEL = 1;  // Switch to INTOSC2
    sysCtrlRegs->CLKCTL.bit.XCLKINOFF = 1;      // Turn off XCLKIN
    sysCtrlRegs->CLKCTL.bit.XTALOSCOFF = 1;     // Turn off XTALOSC
    sysCtrlRegs->CLKCTL.bit.OSCCLKSRCSEL = 1;   // Switch to Internal Oscillator 2
    sysCtrlRegs->CLKCTL.bit.WDCLKSRCSEL = 0;    // Clock Watchdog off of INTOSC1 always
    sysCtrlRegs->CLKCTL.bit.INTOSC1OFF = 0;     // Leave INTOSC1 on
    EDIS;
}

// This function switches to External CRYSTAL oscillator and turns off all other clock
// sources to minimize power consumption. This option may not be available on all
// device packages

void setXTALOsc (void)
{
     EALLOW;
     sysCtrlRegs->CLKCTL.bit.XTALOSCOFF = 0;     // Turn on XTALOSC
     sysCtrlRegs->CLKCTL.bit.XCLKINOFF = 1;      // Turn off XCLKIN
     sysCtrlRegs->CLKCTL.bit.OSCCLKSRC2SEL = 0;  // Switch to external clock
     sysCtrlRegs->CLKCTL.bit.OSCCLKSRCSEL = 1;   // Switch from INTOSC1 to INTOSC2/ext clk
     sysCtrlRegs->CLKCTL.bit.WDCLKSRCSEL = 0;    // Clock Watchdog off of INTOSC1 always
     sysCtrlRegs->CLKCTL.bit.INTOSC2OFF = 1;     // Turn off INTOSC2
     sysCtrlRegs->CLKCTL.bit.INTOSC1OFF = 0;     // Leave INTOSC1 on
     EDIS;
}


// This function switches to External oscillator and turns off all other clock
// sources to minimize power consumption.

void setExtOsc (void)
{
     EALLOW;
     sysCtrlRegs->XCLK.bit.XCLKINSEL = 1;       // 1-GPIO19 = XCLKIN, 0-GPIO38 = XCLKIN
     sysCtrlRegs->CLKCTL.bit.XTALOSCOFF = 1;    // Turn on XTALOSC
     sysCtrlRegs->CLKCTL.bit.XCLKINOFF = 0;     // Turn on XCLKIN
     sysCtrlRegs->CLKCTL.bit.OSCCLKSRC2SEL = 0; // Switch to external clock
     sysCtrlRegs->CLKCTL.bit.OSCCLKSRCSEL = 1;  // Switch from INTOSC1 to INTOSC2/ext clk
     sysCtrlRegs->CLKCTL.bit.WDCLKSRCSEL = 0;   // Clock Watchdog off of INTOSC1 always
     sysCtrlRegs->CLKCTL.bit.INTOSC2OFF = 1;    // Turn off INTOSC2
     sysCtrlRegs->CLKCTL.bit.INTOSC1OFF = 0;     // Leave INTOSC1 on
     EDIS;
}
#endif /* SYSTEMDRIVER_H_ */
