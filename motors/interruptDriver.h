/*
 * interruptDriver.h
 *
 *  Created on: 14 ���. 2016
 *      Author: Mishko Lytvyn
 */

#include "F2806x_PieCtrl.h"
#include "F2806x_PieVect.h"
#include "F2806x_XIntrupt.h"
#include "F2806x_DefaultISR.h"

#ifndef INTERRUPTDRIVER_H_
#define INTERRUPTDRIVER_H_

#define PIE_REGS_BASE_ADDR (0x00000ce0)
#define PIE_VECT_BASE_ADDR (0x00000d00)
#define XINT_VECT_BASE_ADDR (0x00007070)


PIE_CTRL_REGS* interrupS;
XINTRUPT_REGS* xIntruptRegs;
PIE_VECT_TABLE* interrupSVectTable;


void interrupsInit(void);
void interrupsCtrlInit(void);
void interruptSEnable(void);
void interrupsCpuSetup(void);
void interrupSDefVectTableInit(void);



void interrupsInit(void)
{
	interrupS = (PIE_CTRL_REGS* )PIE_REGS_BASE_ADDR;
	interrupSVectTable = (PIE_VECT_TABLE* )PIE_VECT_BASE_ADDR;
	xIntruptRegs = (XINTRUPT_REGS* )XINT_VECT_BASE_ADDR;
}

void interrupsCtrlInit(void)
{
    // Disable Interrupts at the CPU level:
    DINT;

    // Disable the PIE
    interrupS->PIECTRL.bit.ENPIE = 0;

	// Clear all PIEIER registers:
	interrupS->PIEIER1.all = 0;
	interrupS->PIEIER2.all = 0;
	interrupS->PIEIER3.all = 0;
	interrupS->PIEIER4.all = 0;
	interrupS->PIEIER5.all = 0;
	interrupS->PIEIER6.all = 0;
	interrupS->PIEIER7.all = 0;
	interrupS->PIEIER8.all = 0;
	interrupS->PIEIER9.all = 0;
	interrupS->PIEIER10.all = 0;
	interrupS->PIEIER11.all = 0;
	interrupS->PIEIER12.all = 0;

	// Clear all PIEIFR registers:
	interrupS->PIEIFR1.all = 0;
	interrupS->PIEIFR2.all = 0;
	interrupS->PIEIFR3.all = 0;
	interrupS->PIEIFR4.all = 0;
	interrupS->PIEIFR5.all = 0;
	interrupS->PIEIFR6.all = 0;
	interrupS->PIEIFR7.all = 0;
	interrupS->PIEIFR8.all = 0;
	interrupS->PIEIFR9.all = 0;
	interrupS->PIEIFR10.all = 0;
	interrupS->PIEIFR11.all = 0;
	interrupS->PIEIFR12.all = 0;

}

//---------------------------------------------------------------------------
// EnableInterrupts:
//---------------------------------------------------------------------------
// This function enables the PIE module and CPU interrupts
//
void interruptSEnable()
{
    // Enable the PIE
    interrupS->PIECTRL.bit.ENPIE = 1;
	// Enables PIE to drive a pulse into the CPU
	interrupS->PIEACK.all = 0xFFFF;
	// Enable Interrupts at the CPU level
    EINT;
}

void interrupsCpuSetup(void)
{
	IER |= M_INT1;
//	IER |= M_INT9;
//	IER |= M_INT6;
	IER |= M_INT13;
	IER |= M_INT14;

	xIntruptRegs->XINT1CR.bit.ENABLE = 0;
	xIntruptRegs->XINT1CR.bit.POLARITY = 01;

	xIntruptRegs->XINT2CR.bit.ENABLE = 0;
	xIntruptRegs->XINT2CR.bit.POLARITY = 01;


//	interrupS->PIEIER1.bit.INTx4 = 0;
//	interrupS->PIEIER1.bit.INTx5 = 0;
	interrupS->PIEIER1.bit.INTx7 = 1;
//	interrupS->PIEIER6.bit.INTx1 = 1;
//	interrupS->PIEIER9.bit.INTx1 = 1;

//	interrupS->PIEIER1.bit.INTx1 = 1;

	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM
}

const PIE_VECT_TABLE PieVectTableInit = {

      PIE_RESERVED,  // 1  Reserved space
      PIE_RESERVED,  // 2  Reserved space
      PIE_RESERVED,  // 3  Reserved space
      PIE_RESERVED,  // 4  Reserved space
      PIE_RESERVED,  // 5  Reserved space
      PIE_RESERVED,  // 6  Reserved space
      PIE_RESERVED,  // 7  Reserved space
      PIE_RESERVED,  // 8  Reserved space
      PIE_RESERVED,  // 9  Reserved space
      PIE_RESERVED,  // 10 Reserved space
      PIE_RESERVED,  // 11 Reserved space
      PIE_RESERVED,  // 12 Reserved space
      PIE_RESERVED,  // 13 Reserved space

// Non-Peripheral Interrupts
      INT13_ISR,     // CPU-Timer 1
      INT14_ISR,     // CPU-Timer 2
      DATALOG_ISR,   // Datalogging interrupt
      RTOSINT_ISR,   // RTOS interrupt
      EMUINT_ISR,    // Emulation interrupt
      NMI_ISR,       // Non-maskable interrupt
      ILLEGAL_ISR,   // Illegal operation TRAP
      USER1_ISR,     // User Defined trap 1
      USER2_ISR,     // User Defined trap 2
      USER3_ISR,     // User Defined trap 3
      USER4_ISR,     // User Defined trap 4
      USER5_ISR,     // User Defined trap 5
      USER6_ISR,     // User Defined trap 6
      USER7_ISR,     // User Defined trap 7
      USER8_ISR,     // User Defined trap 8
      USER9_ISR,     // User Defined trap 9
      USER10_ISR,    // User Defined trap 10
      USER11_ISR,    // User Defined trap 11
      USER12_ISR,    // User Defined trap 12

// Group 1 PIE Vectors
      ADCINT1_ISR,     // 1.1 ADC  ADC - make rsvd1_1 if ADCINT1 is wanted in Group 10 instead.
      ADCINT2_ISR,     // 1.2 ADC  ADC - make rsvd1_2 if ADCINT2 is wanted in Group 10 instead.
      rsvd_ISR,        // 1.3
      XINT1_ISR,       // 1.4 External Interrupt
      XINT2_ISR,       // 1.5 External Interrupt
      ADCINT9_ISR,     // 1.6 ADC Interrupt 9
      TINT0_ISR,       // 1.7 Timer 0
      WAKEINT_ISR,     // 1.8 WD, Low Power

// Group 2 PIE Vectors
      EPWM1_TZINT_ISR, // 2.1 EPWM-1 Trip Zone
      EPWM2_TZINT_ISR, // 2.2 EPWM-2 Trip Zone
      EPWM3_TZINT_ISR, // 2.3 EPWM-3 Trip Zone
      EPWM4_TZINT_ISR, // 2.4 EPWM-4 Trip Zone
      EPWM5_TZINT_ISR, // 2.5 EPWM-5 Trip Zone
      EPWM6_TZINT_ISR, // 2.6 EPWM-6 Trip Zone
      EPWM7_TZINT_ISR, // 2.7 EPWM-7 Trip Zone
      EPWM8_TZINT_ISR, // 2.8 EPWM-8 Trip Zone

// Group 3 PIE Vectors
      EPWM1_INT_ISR,   // 3.1 EPWM-1 Interrupt
      EPWM2_INT_ISR,   // 3.2 EPWM-2 Interrupt
      EPWM3_INT_ISR,   // 3.3 EPWM-3 Interrupt
      EPWM4_INT_ISR,   // 3.4 EPWM-4 Interrupt
      EPWM5_INT_ISR,   // 3.5 EPWM-5 Interrupt
      EPWM6_INT_ISR,   // 3.6 EPWM-6 Interrupt
      EPWM7_INT_ISR,   // 3.7 EPWM-7 Interrupt
      EPWM8_INT_ISR,   // 3.8 EPWM-8 Interrupt

// Group 4 PIE Vectors
      ECAP1_INT_ISR,   // 4.1 ECAP-1
      ECAP2_INT_ISR,   // 4.2 ECAP-2
      ECAP3_INT_ISR,   // 4.3 ECAP-3
      rsvd_ISR,   	   // 4.4
      rsvd_ISR,        // 4.5
      rsvd_ISR,        // 4.6
      HRCAP1_INT_ISR,  // 4.7 HRCAP-1
      HRCAP2_INT_ISR,  // 4.8 HRCAP-2

// Group 5 PIE Vectors

      EQEP1_INT_ISR,   // 5.1 EQEP-1
      EQEP2_INT_ISR,   // 5.2 EQEP-2
      rsvd_ISR,   	   // 5.3
      HRCAP3_INT_ISR,  // 5.4 HRCAP-3
      HRCAP4_INT_ISR,  // 5.5 HRCAP-4
      rsvd_ISR,        // 5.6
      rsvd_ISR,        // 5.7
      USB0_INT_ISR,    // 5.8 USB-0

// Group 6 PIE Vectors
      SPIRXINTA_ISR,   // 6.1 SPI-A
      SPITXINTA_ISR,   // 6.2 SPI-A
      SPIRXINTB_ISR,   // 6.3 SPI-B
      SPITXINTA_ISR,   // 6.4 SPI-B
      MRINTA_ISR,      // 6.5 McBSP-A
      MXINTA_ISR,      // 6.6 McBSP-A
      rsvd_ISR,        // 6.7
      rsvd_ISR,        // 6.8

// Group 7 PIE Vectors
      DINTCH1_ISR,     // 7.1 DMA Channel 1
      DINTCH2_ISR,     // 7.2 DMA Channel 2
      DINTCH3_ISR,     // 7.3 DMA Channel 3
      DINTCH4_ISR,     // 7.4 DMA Channel 4
      DINTCH5_ISR,     // 7.5 DMA Channel 5
      DINTCH6_ISR,     // 7.6 DMA Channel 6
      rsvd_ISR,        // 7.7
      rsvd_ISR,        // 7.8

// Group 8 PIE Vectors
      I2CINT1A_ISR,    // 8.1 I2C-A
      I2CINT2A_ISR,    // 8.2 I2C-A
      rsvd_ISR,        // 8.3
      rsvd_ISR,        // 8.4
      rsvd_ISR,        // 8.5
      rsvd_ISR,        // 8.6
      rsvd_ISR,        // 8.7
      rsvd_ISR,        // 8.8

// Group 9 PIE Vectors
      SCIRXINTA_ISR,   // 9.1 SCI-A
      SCITXINTA_ISR,   // 9.2 SCI-A
      SCIRXINTB_ISR,   // 9.3 SCI-B
      SCITXINTB_ISR,   // 9.4 SCI-B
      ECAN0INTA_ISR,   // 9.5 ECAN-A
      ECAN1INTA_ISR,   // 9.6 ECAN-A
      rsvd_ISR,        // 9.7
      rsvd_ISR,        // 9.8

// Group 10 PIE Vectors
      rsvd_ISR,        // 10.1 Can be ADCINT1, but must make ADCINT1 in Group 1 space "reserved".
      rsvd_ISR,        // 10.2 Can be ADCINT2, but must make ADCINT2 in Group 1 space "reserved".
      ADCINT3_ISR,     // 10.3 ADC
      ADCINT4_ISR,     // 10.4 ADC
      ADCINT5_ISR,     // 10.5 ADC
      ADCINT6_ISR,     // 10.6 ADC
      ADCINT7_ISR,     // 10.7 ADC
      ADCINT8_ISR,     // 10.8 ADC

// Group 11 PIE Vectors
      CLA1_INT1_ISR,   // 11.1 CLA1
	  CLA1_INT2_ISR,   // 11.2 CLA1
	  CLA1_INT3_ISR,   // 11.3 CLA1
	  CLA1_INT4_ISR,   // 11.4 CLA1
	  CLA1_INT5_ISR,   // 11.5 CLA1
	  CLA1_INT6_ISR,   // 11.6 CLA1
	  CLA1_INT7_ISR,   // 11.7 CLA1
      CLA1_INT8_ISR,   // 11.8 CLA1

// Group 12 PIE Vectors
      XINT3_ISR,       // 12.1 External Interrupt
      rsvd_ISR,        // 12.2
      rsvd_ISR,        // 12.3
      rsvd_ISR,        // 12.4
      rsvd_ISR,        // 12.5
      rsvd_ISR,        // 12.6
      LVF_ISR,         // 12.7 Latched Overflow
      LUF_ISR          // 12.8 Latched Underflow
};

void interrupSDefVectTableInit(void)
{
	IER = 0x0000;
	IFR = 0x0000;

	int16	i;
	Uint32 *Source = (void *) &PieVectTableInit;
	Uint32 *Dest = (void *) interrupSVectTable;

	// Do not write over first 3 32-bit locations (these locations are
	// initialized by Boot ROM with boot variables)

	Source = Source + 3;
	Dest = Dest + 3;

	EALLOW;
	for(i=0; i < 125; i++)
		*Dest++ = *Source++;
	EDIS;

	// Enable the PIE Vector Table
	interrupS->PIECTRL.bit.ENPIE = 1;

}

// Connected to INT13 of CPU (use MINT13 mask):
// ISR can be used by the user.
__interrupt void INT13_ISR(void)     // INT13 or CPU-Timer1
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void INT14_ISR(void)     // INT14 or CPU-Timer2
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void DATALOG_ISR(void)   // Datalogging interrupt
{
   // Insert ISR Code here

   // Next two lines for debug only to halt the processor here
   // Remove after inserting ISR Code
  __asm ("      ESTOP0");
   for(;;);
}

__interrupt void RTOSINT_ISR(void)   // RTOS interrupt
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void EMUINT_ISR(void)    // Emulation interrupt
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void NMI_ISR(void)       // Non-maskable interrupt
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void ILLEGAL_ISR(void)   // Illegal operation TRAP
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm("          ESTOP0");
  for(;;);

}

__interrupt void USER1_ISR(void)     // User Defined trap 1
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

__interrupt void USER2_ISR(void)     // User Defined trap 2
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

__interrupt void USER3_ISR(void)     // User Defined trap 3
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void USER4_ISR(void)     // User Defined trap 4
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void USER5_ISR(void)     // User Defined trap 5
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void USER6_ISR(void)     // User Defined trap 6
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void USER7_ISR(void)     // User Defined trap 7
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void USER8_ISR(void)     // User Defined trap 8
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void USER9_ISR(void)     // User Defined trap 9
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void USER10_ISR(void)    // User Defined trap 10
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void USER11_ISR(void)    // User Defined trap 11
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void USER12_ISR(void)     // User Defined trap 12
{
 // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

// -----------------------------------------------------------
// PIE Group 1 - MUXed into CPU INT1
// -----------------------------------------------------------
// INT1.1
__interrupt void ADCINT1_ISR(void)   // ADC  (Can also be ISR for INT10.1 when enabled)
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code

 __asm ("      ESTOP0");
  for(;;);

}

// INT1.2
__interrupt void ADCINT2_ISR(void)  // ADC  (Can also be ISR for INT10.2 when enabled)
{

  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code

 __asm("	  ESTOP0");
  for(;;);

}

// INT1.3 - Reserved

// INT1.4
__interrupt void  XINT1_ISR(void)
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

// INT1.5
__interrupt void  XINT2_ISR(void)
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

// INT1.6
__interrupt void  ADCINT9_ISR(void)
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

// INT1.7
__interrupt void  TINT0_ISR(void)      // CPU-Timer 0
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT1.8
__interrupt void  WAKEINT_ISR(void)    // WD, LOW Power
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// -----------------------------------------------------------
// PIE Group 2 - MUXed into CPU INT2
// -----------------------------------------------------------

// INT2.1
__interrupt void EPWM1_TZINT_ISR(void)    // EPWM Trip Zone-1
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT2.2
__interrupt void EPWM2_TZINT_ISR(void)    // EPWM Trip Zone-2
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT2.3
__interrupt void EPWM3_TZINT_ISR(void)    // EPWM Trip Zone-3
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT2.4
__interrupt void EPWM4_TZINT_ISR(void)    // EPWM Trip Zone-4
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT2.5
__interrupt void EPWM5_TZINT_ISR(void)    // EPWM Trip Zone-5
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT2.6
__interrupt void EPWM6_TZINT_ISR(void)    // EPWM Trip Zone-6
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT2.7
__interrupt void EPWM7_TZINT_ISR(void)    // EPWM Trip Zone-7
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT2.8
__interrupt void EPWM8_TZINT_ISR(void)    // EPWM Trip Zone-8
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// -----------------------------------------------------------
// PIE Group 3 - MUXed into CPU INT3
// -----------------------------------------------------------

// INT 3.1
__interrupt void EPWM1_INT_ISR(void)     // EPWM-1
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT3.2
__interrupt void EPWM2_INT_ISR(void)     // EPWM-2
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT3.3
__interrupt void EPWM3_INT_ISR(void)    // EPWM-3
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT3.4
__interrupt void EPWM4_INT_ISR(void)    // EPWM-4
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT3.5
__interrupt void EPWM5_INT_ISR(void)    // EPWM-5
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT3.6
__interrupt void EPWM6_INT_ISR(void)    // EPWM-6
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT3.7
__interrupt void EPWM7_INT_ISR(void)    // EPWM-7
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT3.8
__interrupt void EPWM8_INT_ISR(void)    // EPWM-8
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// -----------------------------------------------------------
// PIE Group 4 - MUXed into CPU INT4
// -----------------------------------------------------------

// INT 4.1
__interrupt void ECAP1_INT_ISR(void)    // ECAP-1
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT4.2
__interrupt void ECAP2_INT_ISR(void)    // ECAP-2
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT4.3
__interrupt void ECAP3_INT_ISR(void)    // ECAP-3
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT4.4 - Reserved
// INT4.5 - Reserved
// INT4.6 - Reserved

// INT4.7
__interrupt void HRCAP1_INT_ISR(void)    // HRCAP-1
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT4.8
__interrupt void HRCAP2_INT_ISR(void)    // HRCAP-2
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// -----------------------------------------------------------
// PIE Group 5 - MUXed into CPU INT5
// -----------------------------------------------------------

// INT 5.1
__interrupt void EQEP1_INT_ISR(void)    // EQEP-1
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT5.2
__interrupt void EQEP2_INT_ISR(void)    // EQEP-2
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT5.3 - Reserved

// INT5.4
__interrupt void HRCAP3_INT_ISR(void)    // HRCAP-3
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT5.5
__interrupt void HRCAP4_INT_ISR(void)    // HRCAP-4
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT5.6 - Reserved
// INT5.7 - Reserved

// INT5.8
__interrupt void USB0_INT_ISR(void)    // USB-0
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// -----------------------------------------------------------
// PIE Group 6 - MUXed into CPU INT6
// -----------------------------------------------------------

// INT6.1
__interrupt void SPIRXINTA_ISR(void)    // SPI-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT6.2
__interrupt void SPITXINTA_ISR(void)     // SPI-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT6.3
__interrupt void SPIRXINTB_ISR(void)    // SPI-B
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT6.4
__interrupt void SPITXINTB_ISR(void)     // SPI-B
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT6.5
__interrupt void MRINTA_ISR(void)     // McBSP-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT6.6
__interrupt void MXINTA_ISR(void)     // McBSP-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT6.7 - Reserved
// INT6.8 - Reserved

// -----------------------------------------------------------
// PIE Group 7 - MUXed into CPU INT7
// -----------------------------------------------------------

// INT7.1
__interrupt void DINTCH1_ISR(void)     // DMA Channel 1
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT7.2
__interrupt void DINTCH2_ISR(void)     // DMA Channel 2
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT7.3
__interrupt void DINTCH3_ISR(void)     // DMA Channel 3
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT7.4
__interrupt void DINTCH4_ISR(void)     // DMA Channel 4
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT7.5
__interrupt void DINTCH5_ISR(void)     // DMA Channel 5
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT7.6
__interrupt void DINTCH6_ISR(void)     // DMA Channel 6
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT7.7 - Reserved
// INT7.8 - Reserved

// -----------------------------------------------------------
// PIE Group 8 - MUXed into CPU INT8
// -----------------------------------------------------------

// INT8.1
__interrupt void I2CINT1A_ISR(void)     // I2C-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT8.2
__interrupt void I2CINT2A_ISR(void)     // I2C-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT8.3 - Reserved
// INT8.4 - Reserved
// INT8.5 - Reserved
// INT8.6 - Reserved
// INT8.7 - Reserved
// INT8.8 - Reserved

// -----------------------------------------------------------
// PIE Group 9 - MUXed into CPU INT9
// -----------------------------------------------------------

// INT9.1
__interrupt void SCIRXINTA_ISR(void)     // SCI-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

// INT9.2
__interrupt void SCITXINTA_ISR(void)     // SCI-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

// INT9.3
__interrupt void SCIRXINTB_ISR(void)     // SCI-B
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

// INT9.4
__interrupt void SCITXINTB_ISR(void)     // SCI-B
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

// INT9.5
__interrupt void ECAN0INTA_ISR(void)     // ECAN-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

// INT9.6
__interrupt void ECAN1INTA_ISR(void)     // ECAN-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

// INT9.7 - Reserved
// INT9.8 - Reserved

// -----------------------------------------------------------
// PIE Group 10 - MUXed into CPU INT10
// -----------------------------------------------------------

// INT10.1 - Reserved or ADCINT1_ISR
// INT10.2 - Reserved or ADCINT2_ISR

// INT10.3
__interrupt void ADCINT3_ISR(void)    // ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT10.4
__interrupt void ADCINT4_ISR(void)    // ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT10.5
__interrupt void ADCINT5_ISR(void)    // ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT10.6
__interrupt void ADCINT6_ISR(void)    // ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT10.7
__interrupt void ADCINT7_ISR(void)    // ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT10.8
__interrupt void ADCINT8_ISR(void)    // ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}


// -----------------------------------------------------------
// PIE Group 11 - MUXed into CPU INT11
// -----------------------------------------------------------

// INT11.1
__interrupt void CLA1_INT1_ISR(void)   // MCLA
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code

 __asm ("      ESTOP0");
  for(;;);

}

// INT11.2
__interrupt void CLA1_INT2_ISR(void)  // MCLA
{

  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code

 __asm("	  ESTOP0");
  for(;;);

}

// INT11.3
__interrupt void CLA1_INT3_ISR(void)    // MCLA
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT11.4
__interrupt void CLA1_INT4_ISR(void)    // MCLA
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT11.5
__interrupt void CLA1_INT5_ISR(void)    // MCLA
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT11.6
__interrupt void CLA1_INT6_ISR(void)    // MCLA
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT11.7
__interrupt void CLA1_INT7_ISR(void)    // MCLA
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// INT11.8
__interrupt void CLA1_INT8_ISR(void)    // MCLA
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);
}

// -----------------------------------------------------------
// PIE Group 12 - MUXed into CPU INT12
// -----------------------------------------------------------

// INT12.1
__interrupt void XINT3_ISR(void)  // External interrupt 3
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

// INT12.2 - Reserved
// INT12.3 - Reserved
// INT12.4 - Reserved
// INT12.5 - Reserved
// INT12.6 - Reserved

// INT12.7
__interrupt void LVF_ISR(void)  // Latched overflow
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

// INT12.8
__interrupt void LUF_ISR(void)  // Latched underflow
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 __asm ("      ESTOP0");
  for(;;);

}

//---------------------------------------------------------------------------
// Catch All Default ISRs:
//

__interrupt void EMPTY_ISR(void)  // Empty ISR - only does a return.
{

}

__interrupt void PIE_RESERVED(void)  // Reserved space.  For test.
{
 __asm ("      ESTOP0");
  for(;;);
}

__interrupt void rsvd_ISR(void)      // For test
{
 __asm ("      ESTOP0");
  for(;;);
}


#endif /* INTERRUPTDRIVER_H_ */
