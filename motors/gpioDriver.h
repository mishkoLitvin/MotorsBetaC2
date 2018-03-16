/*
 * pwmDriver.h
 *
 *  Created on: 12 вер. 2016
 *      Author: Mishko Lytvyn
 */
#include "gpio.h"
#include "defines.h"


#ifndef PWMDRIVER_H_
#define PWMDRIVER_H_

GPIO_Handle gpioS;

void gpioInit();
void setupGPIOs();

void gpioInit()
{
	gpioS = GPIO_init((void *)GPIO_BASE_ADDR,sizeof(GPIO_Obj));
}

void setupGPIOs()
{
	  // PWM
	  GPIO_setMode(gpioS,GPIO_Number_0,GPIO_0_Mode_EPWM1A);
	  GPIO_setMode(gpioS,GPIO_Number_1,GPIO_1_Mode_EPWM1B);
	  GPIO_setMode(gpioS,GPIO_Number_2,GPIO_2_Mode_EPWM2A);
	  GPIO_setMode(gpioS,GPIO_Number_3,GPIO_3_Mode_EPWM2B);
	  GPIO_setMode(gpioS,GPIO_Number_4,GPIO_4_Mode_EPWM3A);
	  GPIO_setMode(gpioS,GPIO_Number_5,GPIO_5_Mode_EPWM3B);
	  GPIO_setMode(gpioS,GPIO_Number_6,GPIO_6_Mode_EPWM4A);
	  GPIO_setMode(gpioS,GPIO_Number_7,GPIO_7_Mode_EPWM4B);
	  GPIO_setMode(gpioS,GPIO_Number_8,GPIO_8_Mode_EPWM5A);
	  GPIO_setMode(gpioS,GPIO_Number_9,GPIO_9_Mode_EPWM5B);
	  GPIO_setMode(gpioS,GPIO_Number_10,GPIO_10_Mode_EPWM6A);
	  GPIO_setMode(gpioS,GPIO_Number_11,GPIO_11_Mode_EPWM6B);

	  // No Connection
	  GPIO_setMode(gpioS,GPIO_Number_12,GPIO_12_Mode_GeneralPurpose);


	  // OCTWn
	  GPIO_setMode(gpioS,GPIO_Number_13,GPIO_13_Mode_TZ2_NOT);

	  // FAULTn
	  GPIO_setMode(gpioS,GPIO_Number_14,GPIO_14_Mode_TZ3_NOT);

	  // No Connection
	  GPIO_setMode(gpioS,GPIO_Number_15,GPIO_15_Mode_GeneralPurpose);

	  //FAULT_2 @ GPIO16
	  GPIO_setMode(gpioS,MOTOR_2_FAULT,GPIO_16_Mode_GeneralPurpose);
	  GPIO_setLow(gpioS,MOTOR_2_FAULT);
	  GPIO_setDirection(gpioS,MOTOR_2_FAULT,GPIO_Direction_Input);

	  // No Connection
	  GPIO_setMode(gpioS,GPIO_Number_17,GPIO_17_Mode_GeneralPurpose);

	  // No Connection GPIO18
	//  GPIO_setMode(gpioS,GPIO_Number_18,GPIO_18_Mode_GeneralPurpose);
	  // XCLK Out GPIO18
	  GPIO_setMode(gpioS, GPIO_Number_18, GPIO_18_Mode_XCLKOUT);

	  // No Connection
	  GPIO_setMode(gpioS,GPIO_Number_19,GPIO_19_Mode_GeneralPurpose);

	  // MCBSP
	  GPIO_setMode(gpioS,GPIO_Number_20,GPIO_20_Mode_MDXA);
	  GPIO_setMode(gpioS,GPIO_Number_21,GPIO_21_Mode_MDRA);
	  GPIO_setMode(gpioS,GPIO_Number_22,GPIO_22_Mode_MCLKXA);
	  GPIO_setMode(gpioS,GPIO_Number_23,GPIO_23_Mode_MFSXA);

	  //SPI_INT1
	  GPIO_setMode(gpioS,GYRO_INT,GPIO_24_Mode_GeneralPurpose);
	  GPIO_setLow(gpioS,GYRO_INT);
	  GPIO_setDirection(gpioS,GYRO_INT,GPIO_Direction_Input);
	  // No Connection
	  GPIO_setMode(gpioS,GPIO_Number_25,GPIO_25_Mode_GeneralPurpose);
	  //PS_ACK
	  GPIO_setMode(gpioS,PS_ACK,GPIO_26_Mode_GeneralPurpose);
	  GPIO_setLow(gpioS,PS_ACK);
	  GPIO_setDirection(gpioS,PS_ACK,GPIO_Direction_Input);
	  //PS_INT
	  GPIO_setMode(gpioS,GPIO_Number_27,GPIO_27_Mode_GeneralPurpose);
	  //RS422 BCOSU
	  GPIO_setMode(gpioS,GPIO_Number_28,GPIO_28_Mode_SCIRXDA);
	  GPIO_setMode(gpioS,GPIO_Number_29,GPIO_29_Mode_SCITXDA);
	  //No connection
	  GPIO_setMode(gpioS,GPIO_Number_30,GPIO_30_Mode_GeneralPurpose);

	  //FAULT_1 @ GPIO31
	  GPIO_setMode(gpioS,MOTOR_1_FAULT,GPIO_31_Mode_GeneralPurpose);
	  GPIO_setLow(gpioS,MOTOR_1_FAULT);
	  GPIO_setDirection(gpioS,MOTOR_1_FAULT,GPIO_Direction_Input);

	  //No connection
	  GPIO_setMode(gpioS,GPIO_Number_32,GPIO_32_Mode_GeneralPurpose);
	  GPIO_setMode(gpioS,GPIO_Number_33,GPIO_33_Mode_GeneralPurpose);
	  GPIO_setMode(gpioS,GPIO_Number_34,GPIO_34_Mode_GeneralPurpose);

	  // JTAG
	  GPIO_setMode(gpioS,GPIO_Number_35,GPIO_35_Mode_JTAG_TDI);
	  GPIO_setMode(gpioS,GPIO_Number_36,GPIO_36_Mode_JTAG_TMS);
	  GPIO_setMode(gpioS,GPIO_Number_37,GPIO_37_Mode_JTAG_TDO);
	  GPIO_setMode(gpioS,GPIO_Number_38,GPIO_38_Mode_JTAG_TCK);

	  // No Connection
	  GPIO_setMode(gpioS,GPIO_Number_39,GPIO_39_Mode_GeneralPurpose);

	  // LED1
	  GPIO_setMode(gpioS,LED1,GPIO_40_Mode_GeneralPurpose);
	  GPIO_setLow(gpioS,LED1);
	  GPIO_setDirection(gpioS,LED1,GPIO_Direction_Output);

	  // AMC_RST
	  GPIO_setMode(gpioS,PS_RST,GPIO_41_Mode_GeneralPurpose);
	  GPIO_setHigh(gpioS,PS_RST);
	  GPIO_setDirection(gpioS,PS_RST,GPIO_Direction_Output);

	  // LED2
	  GPIO_setMode(gpioS,LED2,GPIO_42_Mode_GeneralPurpose);
	  GPIO_setLow(gpioS,LED2);
	  GPIO_setDirection(gpioS,LED2,GPIO_Direction_Output);

	  // Lock Device
	  GPIO_setMode(gpioS,LD_STEP,GPIO_43_Mode_GeneralPurpose);
	  GPIO_setHigh(gpioS,LD_STEP);
	  GPIO_setDirection(gpioS,LD_STEP,GPIO_Direction_Output);
	  GPIO_setMode(gpioS,LD_DIR,GPIO_44_Mode_GeneralPurpose);
	  GPIO_setHigh(gpioS,LD_DIR);
	  GPIO_setDirection(gpioS,LD_DIR,GPIO_Direction_Output);
	  GPIO_setMode(gpioS,LD_SLEEP,GPIO_50_Mode_GeneralPurpose);
	  GPIO_setHigh(gpioS,LD_SLEEP);
	  GPIO_setDirection(gpioS,LD_SLEEP,GPIO_Direction_Output);
	  GPIO_setMode(gpioS,LD_FAULT,GPIO_51_Mode_GeneralPurpose);
	  GPIO_setHigh(gpioS,LD_FAULT);
	  GPIO_setDirection(gpioS,LD_FAULT,GPIO_Direction_Input);

	  //PS_A0
	  GPIO_setMode(gpioS,PS_A0,GPIO_52_Mode_GeneralPurpose);
	  GPIO_setHigh(gpioS, PS_A0);
	  GPIO_setDirection(gpioS,PS_A0,GPIO_Direction_Output);

	  //PS_A1
	  GPIO_setMode(gpioS,PS_A1,GPIO_53_Mode_GeneralPurpose);
	  GPIO_setHigh(gpioS, PS_A1);
	  GPIO_setDirection(gpioS,PS_A1,GPIO_Direction_Output);


	  // SPIA
	  GPIO_setMode(gpioS,GPIO_Number_54,GPIO_54_Mode_SPISIMOA);
	  GPIO_setMode(gpioS,GPIO_Number_55,GPIO_55_Mode_SPISOMIA);
	  GPIO_setMode(gpioS,GPIO_Number_56,GPIO_56_Mode_SPICLKA);
	  GPIO_setMode(gpioS,GPIO_Number_57,GPIO_57_Mode_SPISTEA_NOT);

	  // LED3
	  GPIO_setMode(gpioS,LED3,GPIO_58_Mode_GeneralPurpose);
	  GPIO_setLow(gpioS,LED3);
	  GPIO_setDirection(gpioS,LED3,GPIO_Direction_Output);
	  return;
}

void setupGPIOsInterrupt()
{
	GPIO_setExtInt(gpioS, PS_ACK, CPU_ExtIntNumber_1);
	GPIO_setExtInt(gpioS, GYRO_INT, CPU_ExtIntNumber_2);


}

#endif /* PWMDRIVER_H_ */
