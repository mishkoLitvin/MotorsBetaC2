/*
 * timerDriver.h
 *
 *  Created on: 14 ���. 2016
 *      Author: Mishko Lytvyn
 */

#include "F2806x_CpuTimers.h"

#ifndef TIMERDRIVER_H_
#define TIMERDRIVER_H_

#define TIMER0_BASE_ADDR   (0x00000c00)
#define TIMER1_BASE_ADDR   (0x00000c08)
#define TIMER2_BASE_ADDR   (0x00000c10)

CPUTIMER_REGS* CpuTimer0Regs;
CPUTIMER_REGS* CpuTimer1Regs;
CPUTIMER_REGS* CpuTimer2Regs;

void timerInit();
void timerSConfig();
void ConfigCpuTimer0();
void ConfigCpuTimer1();
void ConfigCpuTimer2();



void timerInit()
{
	CpuTimer0Regs = (CPUTIMER_REGS*) TIMER0_BASE_ADDR;
	CpuTimer1Regs = (CPUTIMER_REGS*) TIMER1_BASE_ADDR;
	CpuTimer2Regs = (CPUTIMER_REGS*) TIMER2_BASE_ADDR;
}

void timerSConfig()
{
	ConfigCpuTimer0();
	ConfigCpuTimer1();
	ConfigCpuTimer2();

	StartCpuTimer0();
	StartCpuTimer1();
	StartCpuTimer2();
}
void ConfigCpuTimer0()
{
	int Freq = 90;
	long Period = 500;
	Uint32 PeriodInClocks = (long) (Freq * Period);
	CpuTimer0Regs->PRD.all = PeriodInClocks - 1; // Counter decrements PRD+1 times each period

	// Set pre-scale counter to divide by 1 (SYSCLKOUT):
	CpuTimer0Regs->TPR.all  = 0;
	CpuTimer0Regs->TPRH.all  = 0;

	// Initialize timer control register:
	CpuTimer0Regs->TCR.bit.TSS = 1;      // 1 = Stop timer, 0 = Start/Restart Timer
	CpuTimer0Regs->TCR.bit.TRB = 1;      // 1 = reload timer
	CpuTimer0Regs->TCR.bit.SOFT = 0;
	CpuTimer0Regs->TCR.bit.FREE = 0;     // Timer Free Run Disabled
	CpuTimer0Regs->TCR.bit.TIE = 1;      // 0 = Disable/ 1 = Enable Timer Interrupt
	;
}
void ConfigCpuTimer1()
{
	int Freq = 90;
	long Period = 125;
	Uint32 PeriodInClocks = (long) (Freq * Period);
	CpuTimer1Regs->PRD.all = PeriodInClocks - 1; // Counter decrements PRD+1 times each period

	// Set pre-scale counter to divide by 1 (SYSCLKOUT):
	CpuTimer1Regs->TPR.all  = 0;
	CpuTimer1Regs->TPRH.all  = 0;

	// Initialize timer control register:
	CpuTimer1Regs->TCR.bit.TSS = 1;      // 1 = Stop timer, 0 = Start/Restart Timer
	CpuTimer1Regs->TCR.bit.TRB = 1;      // 1 = reload timer
	CpuTimer1Regs->TCR.bit.SOFT = 0;
	CpuTimer1Regs->TCR.bit.FREE = 0;     // Timer Free Run Disabled
	CpuTimer1Regs->TCR.bit.TIE = 1;      // 0 = Disable/ 1 = Enable Timer Interrupt
	;
}
void ConfigCpuTimer2()
{
	int Freq = 90;
	long Period = 50;
	Uint32 PeriodInClocks = (long) (Freq * Period);
	CpuTimer2Regs->PRD.all = PeriodInClocks - 1; // Counter decrements PRD+1 times each period

	// Set pre-scale counter to divide by 1 (SYSCLKOUT):
	CpuTimer2Regs->TPR.all  = 0;
	CpuTimer2Regs->TPRH.all  = 0;

	// Initialize timer control register:
	CpuTimer2Regs->TCR.bit.TSS = 1;      // 1 = Stop timer, 0 = Start/Restart Timer
	CpuTimer2Regs->TCR.bit.TRB = 1;      // 1 = reload timer
	CpuTimer2Regs->TCR.bit.SOFT = 0;
	CpuTimer2Regs->TCR.bit.FREE = 0;     // Timer Free Run Disabled
	CpuTimer2Regs->TCR.bit.TIE = 1;      // 0 = Disable/ 1 = Enable Timer Interrupt

}

#endif /* TIMERDRIVER_H_ */
