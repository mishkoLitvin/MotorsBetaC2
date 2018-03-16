#define LED1 GPIO_Number_40
#define LED2 GPIO_Number_42
#define LED3 GPIO_Number_58

#define CPU_Freq PLL_ClkFreq_90_MHz
#define CPU_FreqT 90


#define SPI_MISO GPIO_Number_55
#define SPI_MOSI GPIO_Number_54
#define SPI_CLK  GPIO_Number_56
#define SPI_STEA GPIO_Number_57

#define PS_A0 GPIO_Number_52
#define PS_A1 GPIO_Number_53
#define PS_RST GPIO_Number_41
#define PS_INT GPIO_Number_27
#define PS_ACK GPIO_Number_26

#define GYRO_INT GPIO_Number_24

#define MOTOR_1_FAULT GPIO_Number_31
#define MOTOR_2_FAULT GPIO_Number_16

#define LD_STEP GPIO_Number_43
#define LD_DIR GPIO_Number_44
#define LD_SLEEP GPIO_Number_50
#define LD_FAULT GPIO_Number_51


#define Device_cal (void   (*)(void))0x3D7C80

extern void usDelay(const uint32_t delay_usec)
{
	uint32_t i;
	for(i = delay_usec*10; i>0;)  i--;
}
