/*
 * main.c
 */

#include "memCopy.h"
#include "gpioDriver.h"
#include "adcDriver.h"
#include "clkDriver.h"
#include "mcbspDriver.h"
#include "spiDriverMAX.h"
#include "sciDriver.h"
#include "systemDriver.h"
#include "interruptDriver.h"
#include "timerDriver.h"
#include "pwmDriver.h"
#include "scidata.h"

#include "motorDriver.h"
#include "pid_reg3.h"

#include "stdlib.h"


void main(void);

//extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

McBSPdata mcbspData, uP, uM;

SPIData spiModule;

PIE_VECT_TABLE* interrupSVectTable;

long adcRes[16];
PWMValues pwmValues;

MotorData motor0, motor1;

__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void GPIO26_XINT1_PS_isr(void);
__interrupt void GPIO24_XINT2_GYRO_isr(void);
__interrupt void SCI_RX_isr(void);

__interrupt void adc_readAll(void);

long j, jj;
int ch;
unsigned long timer2cnt;
int stepsCount;
double temp;
double speed0;
float vel;
long i;
int end, flGyro, flPS;
long gCnt;

int sciTx, sciRx;
int sciRxC[4];
unsigned int sciCounter, sciSendCnt;
SAAD_FRAME saadFrame;

float alpha;
float calVect[2][25];
int cal;
int mode;
int send;

float kDif, difPart, posDelta;
float p, dp;

PIDREG3 pid = PIDREG3_DEFAULTS;

void main(void) {

//	memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);

	systemInit();
	systemSetup();

	gpioInit();
	setupGPIOs();
	setupGPIOsInterrupt();


	mcbspaInit();
	mcbspaSetup();
	mcbspSetupSpi();
	AMCconf();

	setSPIData(&spiModule);
	spiInit();
	spiSetup();
	spiGyroConf();
	temp = gyroVerify();
//	gyroSelfTest(spiHandle);
//	gyroUpdateData(spiHandle);

	sciaInit();
	sciaSetup();

	timerInit();
	timerSConfig();

	pwmSInit();
	pwmSetup();

	interrupsInit();
	interrupsCtrlInit();

	EALLOW;  // This is needed to write to EALLOW protected registers
	interrupSVectTable->TINT0 = &cpu_timer0_isr;
	interrupSVectTable->TINT1 = &cpu_timer1_isr;
	interrupSVectTable->TINT2 = &cpu_timer2_isr;
	interrupSVectTable->XINT1 = &GPIO26_XINT1_PS_isr;
	interrupSVectTable->XINT2 = &GPIO24_XINT2_GYRO_isr;
	interrupSVectTable->SCIRXINTA = &SCI_RX_isr;
//	interrupSVectTable->ADCINT1 = &adc_readAll;
	EDIS;

	interruptSEnable();
	interrupsCpuSetup();

	adcInit();
	ADC_config();

	motor0.index = 0;
	motor0.direction = 1;
	motor0.phaseZero = 1.1;
	motor0.phaseTime = motor0.phaseZero;
	motor0.pwmData = getMaxPWMVal(0)*0.20;
	motor0.velocity = 0;
	motor0.polesCount = 10.;

	motor1.index = 1;
	motor1.direction = 1;
	motor1.phaseZero = 3.02;
	motor1.phaseTime = motor1.phaseZero;
	motor1.pwmData = getMaxPWMVal(0)*0.20;
	motor1.velocity = 0;
	motor1.polesCount = 10.;

	motor0.leftPos = 0.40;
	motor0.rightPos = -0.27;

//	motor0.phaseZero = 6.;
//	motor1.phaseZero = 3.1;
//	motor0.polesCount = 32.757/2.;
//	motor1.polesCount = 32.757/2.;
//
//	motor0.leftPos = 1.50;
//	motor0.rightPos = -0.85;

	vel = 0;

	timer2cnt = 0;
	stepsCount = 10;
	speed0 = 0;

	for(i = 0; i<1000;)i++;

	pwmValues = calcPWM(&motor0);

	flGyro = 1;
	flPS = 1;
	gyroUpdateData();
	j = 0;

	sciTx = 0xAA;
	sciCounter = 0;
	sciSendCnt = 0;
	send = 0;

	temp = 0;

	mode = 2;
	cal = 0;

	kDif = 0.01;
	dp = 0;
	p = 0;


	motor0.phasePosition = 0;
	motor0.phasePosStep = 0;

	motor1.phasePosition = 0;
	motor1.phasePosStep = 0;

	end = 1;

	for(;;)
	{
//		GPIO_setHigh(gpioS, LED3);
		for(i=0;i<16;i++)
			adcRes[i] = adcRead(i);
//		GPIO_setLow(gpioS, LED3);

		for(i=0;i<50000;)i++;

		if(send>0)
		{
//			sprintf(adc, "%i", uM.data1);
			sciaWrite(saadFrame.START_BIT.bit.START_0);
			sciaWrite(saadFrame.START_BIT.bit.START_1);
			sciaWrite(saadFrame.START_BIT.bit.START_2);
			sciaWrite(saadFrame.START_BIT.bit.START_3);
			for(;;)
				if(sciaRegs->SCIFFTX.bit.TXFFST == 0) break;
			sciaWrite(saadFrame.COMMAND_BYTE.bit.COMMAND_H);
			sciaWrite(saadFrame.COMMAND_BYTE.bit.COMMAND_L);
			sciaWrite(saadFrame.POSITION.bit.POSITION_H);
			sciaWrite(saadFrame.POSITION.bit.POSITION_L);
			for(;;)
				if(sciaRegs->SCIFFTX.bit.TXFFST == 0) break;
			sciaWrite(saadFrame.VELOCITY.bit.VELOCITY_H);
			sciaWrite(saadFrame.VELOCITY.bit.VELOCITY_L);
			sciaWrite(saadFrame.CTRLSUM.bit.CTRLSUM_H);
			sciaWrite(saadFrame.CTRLSUM.bit.CTRLSUM_L);

			send = 0;
		}

		if(flGyro == 1)
		{
			spiWrite( 0x02, 0x01);//data update freq
			for(i=0;i<500;)i++;
			gyroVerify();
			for(i=0;i<500;)i++;
			gyroUpdateData();
//			fl = 0;
		}

		if(flPS == 1)
		{
			mcbspData = mcbsp_read(0x00);
			if(mcbspData.data1&0x1000)
				uP = mcbsp_read(0x1D);
			if(mcbspData.data1&0x2000)
				uM = mcbsp_read(0x1F);
		}

		if(cal == 1)
		{
//			end = 1;
			for(i=0;i<25;i++)
			{
				calVect[0][i] = i*0.5;
				motor0.phaseTime = i*0.5+motor0.phaseZero;
				motor1.phaseTime = i*0.5+motor1.phaseZero;
				for(j=0;j<200;j++)
					for(jj=0;jj<50000;)jj++;
				calVect[1][i] = alpha;
			}
			end = 0;
			cal = 0;
		}



		if(end==1)
		{
//			motor0.phaseTime = temp+motor0.phaseZero;
//			motor1.phaseTime = temp+motor1.phaseZero;

			stopMotor(&motor0);
			stopMotor(&motor1);
		}
		else
		{
			motor0.pwmData = getMaxPWMVal(0)*0.30;
			motor1.pwmData = getMaxPWMVal(0)*0.30;

		}

	}

}

__interrupt void cpu_timer0_isr(void)
{
//	GPIO_setHigh(gpioS, LED2);


//		motor0.phaseTime += pid.Out/10.;
	motor0.phaseTime+=dp;
		if(motor0.phaseTime >=2.*PI)motor0.phaseTime-=2.*PI;
		if(motor0.phaseTime <=0)motor0.phaseTime+=2.*PI;

//		motor1.phaseTime += pid.Out/10.;
		motor1.phaseTime+=dp;
		if(motor1.phaseTime >=2.*PI)motor1.phaseTime-=2.*PI;
		if(motor1.phaseTime <=0)motor1.phaseTime+=2.*PI;
		//		motor0.pwmData = 300+1E7*fabs(difPart);
		//		motor1.pwmData = 300+1E7*fabs(difPart);

	setPWMValuesArr(calcPWM(&motor0));
	setPWMValuesArr(calcPWM(&motor1));

	if(mode == 1)
		timer2cnt++;
//	GPIO_setLow(gpioS, LED2);
	interrupS->PIEACK.all = PIEACK_GROUP1;

}

__interrupt void cpu_timer1_isr(void)
{
	GPIO_setHigh(gpioS, LED3);


	mcbspData = mcbsp_read(0x00);
	if(mcbspData.data1&0x1000)
		uP = mcbsp_read(0x1D);
	if(mcbspData.data1&0x2000)
		uM = mcbsp_read(0x1F);

	alpha = (uP.data1*1.-uM.data1*1.)/1./(uP.data1*1.+uM.data1*1.);

	if((alpha>(motor0.leftPos)))
	{
		dp = -1.*temp;
		dp = -1.*temp;
	}
	if((alpha<(motor0.rightPos)))
	{
		dp = 1.*temp;
		dp = 1.*temp;
	}

	flPS = 0;
	p += dp;

//	pid.Ref = p;
//	pid.Fdb = alpha;


//	pid_reg3_calc(&pid);

	GPIO_setLow(gpioS, LED3);

}

__interrupt void cpu_timer2_isr(void)
{

}

__interrupt void GPIO26_XINT1_PS_isr(void)
{

	interrupS->PIEACK.all = PIEACK_GROUP1;

}

__interrupt void GPIO24_XINT2_GYRO_isr(void)
{
//	gyroVerify();
	gyroUpdateData();
	interrupS->PIEACK.all = PIEACK_GROUP1;
	xIntruptRegs->XINT2CTR = 0;
	flGyro = 0;
}

__interrupt void SCI_RX_isr(void)
{
	if(sciaRegs->SCIFFRX.bit.RXFFST==4)
	{
		sciRxC[0] = sciaRead();
		sciRxC[1] = sciaRead();
		sciRxC[2] = sciaRead();
		sciRxC[3] = sciaRead();
		if((sciRxC[0]==0x55)&(sciRxC[1]==0x55)
				&(sciRxC[2]==0x55)&(sciRxC[3]==0x55))
		{
			saadFrame.START_BIT.bit.START_0 = sciRxC[0];
			saadFrame.START_BIT.bit.START_1 = sciRxC[1];
			saadFrame.START_BIT.bit.START_2 = sciRxC[2];
			saadFrame.START_BIT.bit.START_3 = sciRxC[3];
			sciCounter = 1;
		}
		else
			if(sciCounter == 1)
			{
				saadFrame.COMMAND_BYTE.bit.COMMAND_H = sciRxC[0];
				saadFrame.COMMAND_BYTE.bit.COMMAND_L = sciRxC[1];
				saadFrame.POSITION.bit.POSITION_H = sciRxC[2];
				saadFrame.POSITION.bit.POSITION_L = sciRxC[3];
				sciCounter = 2;
			}
			else
				if(sciCounter == 2)
				{
					saadFrame.VELOCITY.bit.VELOCITY_H = sciRxC[0];
					saadFrame.VELOCITY.bit.VELOCITY_L = sciRxC[1];
					saadFrame.CTRLSUM.bit.CTRLSUM_H = sciRxC[2];
					saadFrame.CTRLSUM.bit.CTRLSUM_L = sciRxC[3];
					sciCounter = 0;
					send = 1;
				}
	}

//	sscanf(sciRxC, "%d", &sciRx);
	sciaRegs->SCIFFRX.bit.RXFFINTCLR = 1;
	interrupS->PIEACK.all = PIEACK_GROUP9;
}


void adc_readAll(void)
{

	adcRegs->ADCINTFLGCLR.bit.ADCINT1 = 1;		//Clear ADCINT1 flag reinitialize for next SOC
	interrupS->PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

}
