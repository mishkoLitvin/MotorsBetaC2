/*
 * pwmDriver.h
 *
 *  Created on: 16 вер. 2016
 *      Author: Mishko Lytvyn
 */

#define PWM_ePWM1_BASE_ADDR          (0x00006800)
#define PWM_ePWM2_BASE_ADDR          (0x00006840)
#define PWM_ePWM3_BASE_ADDR          (0x00006880)
#define PWM_ePWM4_BASE_ADDR          (0x000068C0)
#define PWM_ePWM5_BASE_ADDR          (0x00006900)
#define PWM_ePWM6_BASE_ADDR          (0x00006940)
#define PWM_ePWM7_BASE_ADDR          (0x00006980)
#define PWM_ePWM8_BASE_ADDR          (0x000069C0)

#include "F2806x_EPwm.h"
#include "F2806x_EPwm_defines.h"
#include "clkDriver.h"


typedef struct PWMValues_
{
	int index;
	int pwmV[3];
}PWMValues;


PWM_Handle pwmHandle[6];
EPWM_REGS *pwmRegS[6];

void pwmSInit(void);
void pwmSetup();
void setPWMValues(int channel, int value);
void setPWMValuesArr(PWMValues valueArr);
int getMaxPWMVal(int channel);

void pwmSInit(void)
{
	pwmRegS[0] = (EPWM_REGS*) PWM_ePWM1_BASE_ADDR;
	pwmRegS[1] = (EPWM_REGS*) PWM_ePWM2_BASE_ADDR;
	pwmRegS[2] = (EPWM_REGS*) PWM_ePWM3_BASE_ADDR;
	pwmRegS[3] = (EPWM_REGS*) PWM_ePWM4_BASE_ADDR;
	pwmRegS[4] = (EPWM_REGS*) PWM_ePWM5_BASE_ADDR;
	pwmRegS[5] = (EPWM_REGS*) PWM_ePWM6_BASE_ADDR;

	return;
}

//void pwmSetup()
//{
//	int i;
//	for(i=0;i<3;i++)
//	{
//		pwmRegS[i]->TBPRD = 1000; // Period = 601 TBCLK counts
//		pwmRegS[i]->CMPA.half.CMPA = 200; // Compare A = 200 TBCLK counts
//		pwmRegS[i]->CMPB = 400; // Compare B = 400 TBCLK counts
//		pwmRegS[i]->TBPHS.all = 0; // Set Phase register to zero
//		pwmRegS[i]->TBCTR = 0; // clear TB counter
//		pwmRegS[i]->TBCTL.bit.CTRMODE = 00;
//		pwmRegS[i]->TBCTL.bit.PHSEN = 0; // Phase loading disabled
//		pwmRegS[i]->TBCTL.bit.PRDLD = 0;
//		pwmRegS[i]->TBCTL.bit.SYNCOSEL = 11;
//		pwmRegS[i]->TBCTL.bit.HSPCLKDIV = 001; // TBCLK = SYSCLKOUT/2
//		pwmRegS[i]->TBCTL.bit.CLKDIV = 00;    //div sysclk/1
//		pwmRegS[i]->CMPCTL.bit.SHDWAMODE = 0;
//		pwmRegS[i]->CMPCTL.bit.SHDWBMODE = 0;
//		pwmRegS[i]->CMPCTL.bit.LOADAMODE = 00; // load on TBCTR = Zero
//		pwmRegS[i]->CMPCTL.bit.LOADBMODE = 00; // load on TBCTR = Zero
//		pwmRegS[i]->AQCTLA.bit.CAU = 01;
//		pwmRegS[i]->AQCTLA.bit.ZRO = 10;
//		pwmRegS[i]->AQCTLB.bit.CBU = 01;
//		pwmRegS[i]->AQCTLB.bit.ZRO = 11;
//	}
//}

void pwmSetupWTest()
{

	pwmRegS[0]->ETSEL.bit.SOCAEN = 1;
	pwmRegS[0]->ETSEL.bit.SOCASEL = 1;
	pwmRegS[0]->ETPS.bit.SOCAPRD = 3;

	// EPWM Module 1 config
	pwmRegS[0]->TBPRD = 900; // Period = 1200 TBCLK counts
	pwmRegS[0]->TBPHS.half.TBPHS = 0; // Set Phase register to zero
	pwmRegS[0]->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	pwmRegS[0]->TBCTL.bit.PHSEN = TB_DISABLE; // Master module
	pwmRegS[0]->TBCTL.bit.PRDLD = TB_SHADOW;
	pwmRegS[0]->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // Sync down-stream module
	pwmRegS[0]->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	pwmRegS[0]->CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;
	pwmRegS[0]->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	pwmRegS[0]->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	pwmRegS[0]->AQCTLA.bit.CAU = AQ_CLEAR; // set actions for EPWM1A
	pwmRegS[0]->AQCTLA.bit.CAD = AQ_SET;
	pwmRegS[0]->AQCSFRC.bit.CSFB = 2; // set actions for EPWM1B
	//	pwmRegS[0]->AQCTLB.bit.CBD = AQ_SET;

	// EPWM Module 2 config
	pwmRegS[1]->TBPRD = 900; // Period = 1200 TBCLK counts
	pwmRegS[1]->TBPHS.half.TBPHS = 0; // Set Phase register to zero
	pwmRegS[1]->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	pwmRegS[1]->TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
	pwmRegS[1]->TBCTL.bit.PRDLD = TB_SHADOW;
	pwmRegS[1]->TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
	pwmRegS[1]->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	pwmRegS[1]->CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;
	pwmRegS[1]->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	pwmRegS[1]->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	pwmRegS[1]->AQCTLA.bit.CAU = AQ_CLEAR; // set actions for EPWM2A
	pwmRegS[1]->AQCTLA.bit.CAD = AQ_SET;
	pwmRegS[1]->AQCSFRC.bit.CSFB = 2; // set actions for EPWM2B
	//	pwmRegS[1]->AQCTLB.bit.CBD = AQ_SET;

	// EPWM Module 3 config
	pwmRegS[2]->TBPRD = 900; // Period = 1200 TBCLK counts
	pwmRegS[2]->TBPHS.half.TBPHS = 0; // Set Phase register to zero
	pwmRegS[2]->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	pwmRegS[2]->TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
	pwmRegS[2]->TBCTL.bit.PRDLD = TB_SHADOW;
	pwmRegS[2]->TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
	pwmRegS[2]->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	pwmRegS[2]->CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;
	pwmRegS[2]->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	pwmRegS[2]->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	pwmRegS[2]->AQCTLA.bit.CAU = AQ_CLEAR; // set actions for EPWM3A
	pwmRegS[2]->AQCTLA.bit.CAD = AQ_SET;
	pwmRegS[2]->AQCSFRC.bit.CSFB = 2; // set actions for pwmRegS[2]->
	//	pwmRegS[2]->AQCTLB.bit.CBD = AQ_SET;

	// EPWM Module 1 config
	pwmRegS[3]->TBPRD = 900; // Period = 1200 TBCLK counts
	pwmRegS[3]->TBPHS.half.TBPHS = 0; // Set Phase register to zero
	pwmRegS[3]->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	pwmRegS[3]->TBCTL.bit.PHSEN = TB_ENABLE; // Master module
	pwmRegS[3]->TBCTL.bit.PRDLD = TB_SHADOW;
	pwmRegS[3]->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // Sync down-stream module
	pwmRegS[3]->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	pwmRegS[3]->CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;
	pwmRegS[3]->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	pwmRegS[3]->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	pwmRegS[3]->AQCTLA.bit.CAU = AQ_CLEAR; // set actions for EPWM1A
	pwmRegS[3]->AQCTLA.bit.CAD = AQ_SET;
	pwmRegS[3]->AQCSFRC.bit.CSFB = 2; // set actions for EPWM1B
	//	pwmRegS[0]->AQCTLB.bit.CBD = AQ_SET;

	// EPWM Module 2 config
	pwmRegS[4]->TBPRD = 900; // Period = 1200 TBCLK counts
	pwmRegS[4]->TBPHS.half.TBPHS = 0; // Set Phase register to zero
	pwmRegS[4]->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	pwmRegS[4]->TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
	pwmRegS[4]->TBCTL.bit.PRDLD = TB_SHADOW;
	pwmRegS[4]->TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
	pwmRegS[4]->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	pwmRegS[4]->CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;
	pwmRegS[4]->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	pwmRegS[4]->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	pwmRegS[4]->AQCTLA.bit.CAU = AQ_CLEAR; // set actions for EPWM2A
	pwmRegS[4]->AQCTLA.bit.CAD = AQ_SET;
	pwmRegS[4]->AQCSFRC.bit.CSFB = 2; // set actions for EPWM2B
	//	pwmRegS[1]->AQCTLB.bit.CBD = AQ_SET;

	// EPWM Module 3 config
	pwmRegS[5]->TBPRD = 900; // Period = 1200 TBCLK counts
	pwmRegS[5]->TBPHS.half.TBPHS = 0; // Set Phase register to zero
	pwmRegS[5]->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	pwmRegS[5]->TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
	pwmRegS[5]->TBCTL.bit.PRDLD = TB_SHADOW;
	pwmRegS[5]->TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
	pwmRegS[5]->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	pwmRegS[5]->CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;
	pwmRegS[5]->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	pwmRegS[5]->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	pwmRegS[5]->AQCTLA.bit.CAU = AQ_CLEAR; // set actions for EPWM3A
	pwmRegS[5]->AQCTLA.bit.CAD = AQ_SET;
	pwmRegS[5]->AQCSFRC.bit.CSFB = 2; // set actions for pwmRegS[2]->
	//	pwmRegS[2]->AQCTLB.bit.CBD = AQ_SET;
}

void setPWMValues(int channel, int value)
{
	pwmRegS[channel]->CMPA.half.CMPA = value; // adjust duty for output EPWM1A only
	pwmRegS[channel]->CMPB = pwmRegS[channel]->TBPRD-1;
//	pwmRegS[channel]->CMPB = pwmRegS[channel]->TBPRD-1;;
}

void disablePWM()
{
	pwmRegS[0]->CMPA.half.CMPA = 0;
	pwmRegS[1]->CMPA.half.CMPA = 0;
	pwmRegS[2]->CMPA.half.CMPA = 0;
	pwmRegS[3]->CMPA.half.CMPA = 0;
	pwmRegS[4]->CMPA.half.CMPA = 0;
	pwmRegS[5]->CMPA.half.CMPA = 0;
}

void setPWMValuesArr(PWMValues valueArr)
{
	int i;
	for(i=0;i<3; i++)
		setPWMValues(valueArr.index*3+i,valueArr.pwmV[i]);
}

int getMaxPWMVal(int channel)
{
	return pwmRegS[channel]->TBPRD-1;
}


