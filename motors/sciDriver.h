/*
 * sciDriver.h
 *
 *  Created on: 19 זמגע. 2016
 *      Author: Mishko Lytvyn
 */

#ifndef SCIDRIVER_H_
#define SCIDRIVER_H_

#include "F2806x_Sci.h"

#define SCIA_BASE_ADDR (0x00007050)

SCI_REGS* sciaRegs;

void sciaInit();
void sciaSetup();





void sciaInit()
{
	sciaRegs = (SCI_REGS*) SCIA_BASE_ADDR;
}

void sciaSetup()
{

 	sciaRegs->SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
 	sciaRegs->SCICCR.bit.PARITYENA = 1;
 	sciaRegs->SCICCR.bit.PARITY = 1;//EVEN
 	sciaRegs->SCICCR.bit.STOPBITS = 0; //2 stopbit

	sciaRegs->SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	sciaRegs->SCICTL2.all =0x0003;
	sciaRegs->SCICTL2.bit.TXINTENA =0;
	sciaRegs->SCICTL2.bit.RXBKINTENA =0;

    sciaRegs->SCIHBAUD    =0x0000;
    sciaRegs->SCILBAUD    =0x0008;//1 250 000
//    sciaRegs->SCILBAUD    =0x000E;//750 000
	sciaRegs->SCICCR.bit.LOOPBKENA =0; // Enable loop back
	sciaRegs->SCICTL1.all =0x0023;     // Relinquish SCI from Reset

	sciaRegs->SCIFFTX.all=0xE040;
	sciaRegs->SCIFFRX.all=0x2044;
	sciaRegs->SCIFFRX.bit.RXFFIENA = 0;

	sciaRegs->SCIFFCT.all=0x0;
}

int sciaRead()
{
	return sciaRegs->SCIRXBUF.all;
}

void sciaWrite(int a)
{
    sciaRegs->SCITXBUF=a;
}

#endif /* SCIDRIVER_H_ */
