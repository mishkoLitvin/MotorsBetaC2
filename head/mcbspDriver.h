/*
 * mcbspDriver.h
 *
 *  Created on: 30 ����. 2016
 *      Author: Mishko Lytvyn
 *      This file contains variables for setting of the McBSP interface.
 *      This interface is configured as SPI and
 *      used to communicate with the angular position sensor.
 *
 */

#include "F2806x_Mcbsp.h"
#include "F2806x_SysCtrl.h"
#include "scidata.h"
//#include "gpioDriver.h"

#ifndef MCBSPDRIVER_H_
#define MCBSPDRIVER_H_

#define McBSPA_BASE_ADDR   (0x005000)

#define CPU_SPD              80E6
#define MCBSP_SRG_FREQ       CPU_SPD/2   // SRG input is LSPCLK (SYSCLKOUT/2) for examples
#define MCBSP_INIT_DELAY     2*(CPU_SPD/MCBSP_SRG_FREQ)                  // # of CPU cycles in 2 SRG cycles-init delay

#define CLKGDV_VAL           1
#define MCBSP_INIT_DELAY     2*(CPU_SPD/MCBSP_SRG_FREQ)                  // # of CPU cycles in 2 SRG cycles-init delay
#define MCBSP_CLKG_DELAY     2*(CPU_SPD/(MCBSP_SRG_FREQ/(1+CLKGDV_VAL))) // # of CPU cycles in 2 CLKG cycles-init delay

McBSP_REGS* mcbspRegs;

typedef struct McBSPdata_
{
	int data1;
	int data2;
} McBSPdata;

McBSPdata tempData1, tempData2;
SAAD_TESTS saadTestsPs;

void delay_loop(void);
void clkg_delay_loop(void);
void mcbspaInit(void);
void mcbspaSetup();
void mcbspSetupSpi();
void mcbsp_write(int addr, long data);
McBSPdata mcbsp_read(int addr);
Uint32 mcbspAddr();
void AMCconf();



void delay_loop(void)
{
	long      i;
	for (i = 0; i < MCBSP_INIT_DELAY; i++) {} //delay in McBsp init. must be at least 2 SRG cycles
}

void clkg_delay_loop(void)
{
	long      i;
	for (i = 0; i < MCBSP_CLKG_DELAY; i++) {} //delay in McBsp init. must be at least 2 SRG cycles
}

void mcbspaInit(void)
{
	mcbspRegs = (void*)McBSPA_BASE_ADDR;
}

void mcbspaSetup()
{
	// McBSP-A register settings
	asm(" EALLOW");
	mcbspRegs->SPCR2.all=0x0000;        // Reset FS generator, sample rate generator & transmitter
	mcbspRegs->SPCR1.all=0x0000;        // Reset Receiver, Right justify word
	mcbspRegs->SPCR1.bit.DLB = 1;       // Enable loopback mode for test. Comment out for normal McBSP transfer mode.
	mcbspRegs->MFFINT.all=0x0;          // Disable all interrupts
	mcbspRegs->RCR2.all=0x0;            // Single-phase frame, 1 word/frame, No companding  (Receive)
	mcbspRegs->RCR1.all=0x0;
	mcbspRegs->XCR2.all=0x0;            // Single-phase frame, 1 word/frame, No companding  (Transmit)
	mcbspRegs->XCR1.all=0x0;
	mcbspRegs->PCR.bit.FSXM = 1;        // FSX generated internally, FSR derived from an external source
	mcbspRegs->PCR.bit.CLKXM = 1;       // CLKX generated internally, CLKR derived from an external source
	mcbspRegs->SRGR2.bit.CLKSM = 1;     // CLKSM=1 (If SCLKME=0, i/p clock to SRG is LSPCLK)
	mcbspRegs->SRGR2.bit.FPER = 31;     // FPER = 32 CLKG periods
	mcbspRegs->SRGR1.bit.FWID = 0;              // Frame Width = 1 CLKG period
	mcbspRegs->SRGR1.bit.CLKGDV = CLKGDV_VAL;   // CLKG frequency = LSPCLK/(CLKGDV+1)
	delay_loop();                // Wait at least 2 SRG clock cycles
	mcbspRegs->SPCR2.bit.GRST=1; // Enable the sample rate generator
	clkg_delay_loop();           // Wait at least 2 CLKG cycles
	mcbspRegs->SPCR2.bit.XRST=1; // Release TX from Reset
	mcbspRegs->SPCR1.bit.RRST=1; // Release RX from Reset
	mcbspRegs->SPCR2.bit.FRST=1; // Frame Sync Generator reset

	asm(" EDIS");

	return;

}

void mcbspSetupSpi()
{
	mcbspRegs = (McBSP_REGS*)McBSPA_BASE_ADDR;

	for(;;) if(mcbspAddr()==0x5000) break;
	mcbspRegs->SPCR2.all=0x0000;		// Reset FS generator, sample rate generator & transmitter
	mcbspRegs->SPCR1.all=0x0000;		// Reset Receiver, Right justify word, Digital loopback dis.
	mcbspRegs->PCR.all=0x0F08;          //(CLKXM=CLKRM=FSXM=FSRM= 1, FSXP = 1)
	mcbspRegs->SPCR1.bit.CLKSTP = 2;    // Together with CLKXP/CLKRP determines clocking scheme
	mcbspRegs->PCR.bit.CLKXP = 0;		// CPOL = 0, CPHA = 0 rising edge no delay
	mcbspRegs->PCR.bit.CLKRP = 0;
	mcbspRegs->RCR2.bit.RDATDLY=01;     // FSX setup time 1 in master mode. 0 for slave mode (Receive)
	mcbspRegs->XCR2.bit.XDATDLY=01;     // FSX setup time 1 in master mode. 0 for slave mode (Transmit)
	mcbspRegs->RCR1.bit.RWDLEN1=4;     	// 24-bit word
	mcbspRegs->XCR1.bit.XWDLEN1=4;     	// 24-bit word
	mcbspRegs->SRGR2.all=0x2000; 	 	// CLKSM=1, FPER = 1 CLKG periods
	mcbspRegs->SRGR1.all= 0x0008;	    // Frame Width = 1 CLKG period, CLKGDV=8
	mcbspRegs->SPCR2.bit.GRST=1;        // Enable the sample rate generator
	delay_loop();                       // Wait at least 2 SRG clock cycles
	mcbspRegs->SPCR2.bit.XRST=1;        // Release TX from Reset
	mcbspRegs->SPCR1.bit.RRST=1;        // Release RX from Reset
	mcbspRegs->SPCR2.bit.FRST=1;        // Frame Sync Generator reset
	mcbspRegs->SPCR1.bit.DLB=0; 		// Loopback 1-enable 0-disable
	//    EDIS;

	return;
}

void mcbsp_write(int addr, long data)
{
	int j;
	for(j = 0; j<10; ) j++;

	while( mcbspRegs->SPCR2.bit.XRDY != 1 ){}// Master waits until TX data is ready
	mcbspRegs->DXR2.all=addr;
	mcbspRegs->DXR1.all=data;
}

McBSPdata mcbsp_read(int addr)
{
	McBSPdata out;
	addr |= 0x80;
	out.data2 = mcbspRegs->DRR2.all;                      // Read DRR2 first.
	out.data1 = mcbspRegs->DRR1.all;                      // Then read DRR1 to complete receiving of data

	mcbsp_write(addr, 0x0000);
//	mcbsp_write(0, addr);
	while( mcbspRegs->SPCR1.bit.RRDY != 1 )
	{}         // Master waits until RX data is ready
	out.data2 = mcbspRegs->DRR2.all;                      // Read DRR2 first.
	out.data1 = mcbspRegs->DRR1.all;                      // Then read DRR1 to complete receiving of data
	return out;
}

Uint32 mcbspAddr()
{
	return (Uint32) mcbspRegs;
}

long AMCconfArr[64] = {0x0000, 0x00CB, 0x2994, 0xA946,
		0x2A18, 0x6142, 0x8488, 0x4440,
		0xA00A, 0x0048, 0x0021, 0x0000,
		0x1400, 0x0000, 0x0000, 0x0000,
		0x0000, 0x0000, 0x0000, 0x0000,
		0x4000, 0x0008, 0x0080, 0x0104,
		0x0084, 0x0440, 0x8822, 0x40C1,
		0x4449, 0x2492, 0x9295, 0x198D,
		0x2CAB, 0x3356, 0x736B, 0x3CEB,
		0xAEBD, 0x776F, 0xAFBB, 0xDEFB,
		0xEFBF, 0xDF7F, 0xFEF9, 0xFFFF,
		0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
		0xFFFF, 0xFFFF, 0xFFFF, 0x7FFF,
		0xFFF7, 0xFFEF, 0xFFBF, 0x7FDF,
		0x7F7B, 0xEEFB, 0xBDDB, 0xD7CF,
		0x5BCD, 0xD5D5, 0xAD59, 0xACCB
};

void AMCconf()
{
	int i,j;
	GPIO_setHigh(gpioS, PS_RST);
	GPIO_setLow(gpioS,PS_RST);
	for(j = 0; j<1000; ) j++;
	GPIO_setHigh(gpioS, PS_RST);
	GPIO_setHigh(gpioS, PS_A0);
	GPIO_setHigh(gpioS, PS_A1);


	mcbsp_write(0x19, 0x03E7); //config control register to unused default params // set up 999 counts to form out sin

	for(i=0;i<64;i++)
	{
		mcbsp_write(0x1A, AMCconfArr[i]);
	}
	mcbsp_write(0x1B, 0x1C00);//config clock divider register freq = 20MHz

	mcbsp_write(0x01, 0x0000);//config control parametr register #1
	mcbsp_write(0x02, 0x0F7C);//config sinc filter parametr register #1
	mcbsp_write(0x03, 0x631F);//config integrator parametr register #1

	mcbsp_write(0x07, 0x0000);//config control parametr register #2
	mcbsp_write(0x08, 0x0F7C);//config sinc filter parametr register #2
	mcbsp_write(0x09, 0x631F);//config integrator parametr register #2

	//enable power part of PS
	GPIO_setHigh(gpioS, PS_A0);
	GPIO_setLow(gpioS, PS_A1);
//	GPIO_setLow(gpioS, PS_A0);
//	GPIO_setLow(gpioS, PS_A1);

	for (;;)
	{
		mcbsp_write(0x1B, 0x0E00);//config clock divider register to calibrate demodulator #1
		for(j = 0; j<5000; ) j++;
		tempData1 = mcbsp_read(0x1B);

		mcbsp_write(0x1B, 0x0E80);//config clock divider register to calibrate demodulator #2
		for(j = 0; j<5000; ) j++;
		tempData2 = mcbsp_read(0x1B);
		if(((tempData1.data1&(1<<9))==0)&((tempData2.data1&(1<<9))==0))
			break;
	}


//	mcbsp_write(0x1B, 0x0400);
//	for(j = 0; j<1000; ) j++;
	mcbsp_write(0x1B, 0x0C00);
}


#endif /* MCBSPDRIVER_H_ */
