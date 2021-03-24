#ifndef __IO_H
#define __IO_H 		
#include "MY_ST_config.h"
#include "math.h"
#include "stdbool.h"


//IIC_SDA	  PB11
#define IIC_SDA_OUT {RCC->IOPENR|=1<<1;GPIOB->MODER&=~(3<<22);GPIOB->MODER|=1<<22;GPIOB->PUPDR|=1<<22;} 
#define IIC_SDA_IN  {RCC->IOPENR|=1<<1;GPIOB->MODER&=~(3<<22);GPIOB->MODER|=0<<22;} 
#define IIC_SDA_SET GPIOB->ODR|=1<<11
#define IIC_SDA_RESET  GPIOB->ODR&=~(1<<11)
#define IIC_SDA_State ((GPIOB->IDR & 1<<11) == 1<<11)

//IIC_SCL	  PB12
#define IIC_SCL_OUT {RCC->IOPENR|=1<<1;GPIOB->MODER&=~(3<<24);GPIOB->MODER|=1<<24;GPIOB->PUPDR|=1<<24;}  
#define IIC_SCL_IN  {RCC->IOPENR|=1<<1;GPIOB->MODER&=~(3<<24);GPIOB->MODER|=0<<24;} 
#define IIC_SCL_SET GPIOB->ODR|=1<<12
#define IIC_SCL_RESET  GPIOB->ODR&=~(1<<12)
#define IIC_SCL_State ((GPIOB->IDR & 1<<12) == 1<<12)

#if 1
void IIC_Init(void);
void IIC_Start(void);//产生IIC起始信号
void IIC_Stop(void);//产生IIC停止信号
void IIC_Ack(void);//产生ACK应答
void IIC_NAck(void);//不产生ACK应答	
uint8_t IIC_Wait_Ack(void);//等待应答信号到来:1,接收应答失败;0,接收应答成功
void IIC_Send_Byte(uint8_t txd);//IIC发送一个字节; 先发送高位
uint8_t IIC_Read_Byte(unsigned char ack);//读一个字节，可加是否应答位,1加ack，0不加ack 从高位开始读
uint8_t IIC_Write_Byte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t data);//直接写一个字节
uint8_t IIC_ReadMulByte(uint8_t DrvAddr,uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);//读字节
uint8_t IIC_WriteMulByte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);//可一次写多个字节
#endif

//LED_CTRL4	 	PA5
#define LED_4_OUT {RCC->IOPENR|=1<<0;GPIOA->MODER&=~(3<<10);GPIOA->MODER|=1<<10;} 
#define LED_4_SET GPIOA->ODR|=1<<5
#define LED_4_RESET  GPIOA->ODR&=~(1<<5)
#define LED_4_TOG GPIOA->ODR^=1<<5


#if USING_BMP180/***************************以下是BMP180传感器**************************/
/***************************以下是BMP180传感器**************************/
/***************************************************************/
/**\name    I2C ADDRESS DEFINITION OF BMP180       */
/***************************************************************/
/*BMP180 I2C Address*/
#define BMP180_I2C_ADDR                           (0xEE >> 0)

/***************************************************************/
/**\name    ERROR CODE DEFINITIONS    */
/***************************************************************/
#define E_BMP_NULL_PTR                            ((int8_t) - 127)
#define E_BMP_COMM_RES                            ((int8_t) - 1)
#define E_BMP_OUT_OF_RANGE                        ((int8_t) - 2)

/***************************************************************/
/**\name    CONSTANTS       */
/***************************************************************/
#define BMP180_RETURN_FUNCTION_TYPE               int8_t
#define   BMP180_INIT_VALUE                       ((uint8_t)0)
#define   BMP180_INITIALIZE_OVERSAMP_SETTING_U8X  ((uint8_t)0)
#define   BMP180_INITIALIZE_SW_OVERSAMP_U8X       ((uint8_t)0)
#define   BMP180_INITIALIZE_NUMBER_OF_SAMPLES_U8X ((uint8_t)1)
#define   BMP180_GEN_READ_WRITE_DATA_LENGTH       ((uint8_t)1)
#define   BMP180_TEMPERATURE_DATA_LENGTH          ((uint8_t)2)
#define   BMP180_PRESSURE_DATA_LENGTH             ((uint8_t)3)
#define   BMP180_SW_OVERSAMP_U8X                  ((uint8_t)1)
#define   BMP180_OVERSAMP_SETTING_U8X             ((uint8_t)3)
#define   BMP180_2MS_DELAY_U8X                    (2)
#define   BMP180_3MS_DELAY_U8X                    (3)
#define   BMP180_AVERAGE_U8X                      (3)
#define   BMP180_INVALID_DATA                     (0)
#define   BMP180_CHECK_DIVISOR                    (0)
#define   BMP180_DATA_MEASURE                     (3)
#define   BMP180_CALCULATE_TRUE_PRESSURE          (8)
#define   BMP180_CALCULATE_TRUE_TEMPERATURE       (8)
#define BMP180_SHIFT_BIT_POSITION_BY_01_BIT       (1)
#define BMP180_SHIFT_BIT_POSITION_BY_02_BITS      (2)
#define BMP180_SHIFT_BIT_POSITION_BY_04_BITS      (4)
#define BMP180_SHIFT_BIT_POSITION_BY_06_BITS      (6)
#define BMP180_SHIFT_BIT_POSITION_BY_08_BITS      (8)
#define BMP180_SHIFT_BIT_POSITION_BY_11_BITS      (11)
#define BMP180_SHIFT_BIT_POSITION_BY_12_BITS      (12)
#define BMP180_SHIFT_BIT_POSITION_BY_13_BITS      (13)
#define BMP180_SHIFT_BIT_POSITION_BY_15_BITS      (15)
#define BMP180_SHIFT_BIT_POSITION_BY_16_BITS      (16)

/***************************************************************/
/**\name    REGISTER ADDRESS DEFINITION       */
/***************************************************************/
/*register definitions */

#define BMP180_PROM_START__ADDR       (0xAA)
#define BMP180_PROM_DATA__LEN         (22)

#define BMP180_CHIP_ID_REG            (0xD0)
#define BMP180_VERSION_REG            (0xD1)

#define BMP180_CTRL_MEAS_REG          (0xF4)
#define BMP180_ADC_OUT_MSB_REG        (0xF6)
#define BMP180_ADC_OUT_LSB_REG        (0xF7)

#define BMP180_SOFT_RESET_REG         (0xE0)

/* temperature measurement */
#define BMP180_T_MEASURE              (0x2E)

/* pressure measurement*/
#define BMP180_P_MEASURE              (0x34)

/* TO be spec'd by GL or SB*/
#define BMP180_TEMP_CONVERSION_TIME   (5)

#define BMP180_PARAM_MG               (3038)
#define BMP180_PARAM_MH               (-7357)
#define BMP180_PARAM_MI               (3791)

/****************************************************/
/**\name    ARRAY SIZE DEFINITIONS      */
/***************************************************/
#define BMP180_TEMPERATURE_DATA_BYTES (2)
#define BMP180_PRESSURE_DATA_BYTES    (3)
#define BMP180_TEMPERATURE_LSB_DATA   (1)
#define BMP180_TEMPERATURE_MSB_DATA   (0)
#define BMP180_PRESSURE_MSB_DATA      (0)
#define BMP180_PRESSURE_LSB_DATA      (1)
#define BMP180_PRESSURE_XLSB_DATA     (2)

#define BMP180_CALIB_DATA_SIZE        (22)
#define BMP180_CALIB_PARAM_AC1_MSB    (0)
#define BMP180_CALIB_PARAM_AC1_LSB    (1)
#define BMP180_CALIB_PARAM_AC2_MSB    (2)
#define BMP180_CALIB_PARAM_AC2_LSB    (3)
#define BMP180_CALIB_PARAM_AC3_MSB    (4)
#define BMP180_CALIB_PARAM_AC3_LSB    (5)
#define BMP180_CALIB_PARAM_AC4_MSB    (6)
#define BMP180_CALIB_PARAM_AC4_LSB    (7)
#define BMP180_CALIB_PARAM_AC5_MSB    (8)
#define BMP180_CALIB_PARAM_AC5_LSB    (9)
#define BMP180_CALIB_PARAM_AC6_MSB    (10)
#define BMP180_CALIB_PARAM_AC6_LSB    (11)
#define BMP180_CALIB_PARAM_B1_MSB     (12)
#define BMP180_CALIB_PARAM_B1_LSB     (13)
#define BMP180_CALIB_PARAM_B2_MSB     (14)
#define BMP180_CALIB_PARAM_B2_LSB     (15)
#define BMP180_CALIB_PARAM_MB_MSB     (16)
#define BMP180_CALIB_PARAM_MB_LSB     (17)
#define BMP180_CALIB_PARAM_MC_MSB     (18)
#define BMP180_CALIB_PARAM_MC_LSB     (19)
#define BMP180_CALIB_PARAM_MD_MSB     (20)
#define BMP180_CALIB_PARAM_MD_LSB     (21)

struct bmp180_calib_param_t
{
	int16_t ac1; /**<calibration ac1 data*/
	int16_t ac2; /**<calibration ac2 data*/
	int16_t ac3; /**<calibration ac3 data*/
	uint16_t ac4; /**<calibration ac4 data*/
	uint16_t ac5; /**<calibration ac5 data*/
	uint16_t ac6; /**<calibration ac6 data*/
	int16_t b1; /**<calibration b1 data*/
	int16_t b2; /**<calibration b2 data*/
	int16_t mb; /**<calibration mb data*/
	int16_t mc; /**<calibration mc data*/
	int16_t md; /**<calibration md data*/
};
extern struct bmp180_calib_param_t calib_param;
/***************************以上是BMP180传感器**************************/
#endif
extern uint8_t F_TASK_BMP180;
void TASK_BMP180(void);

struct ctrl_state
{
	bool flagmax;//达到设定值上限标志位
	bool flagmin;//达到设定值下限标志位
	uint8_t mode;//控制模式
	uint16_t range;//设定阈值
	int16_t ctrl;//控制输入值	
	float now;//当前输出值
	float set;//设定输出值	
};
void IO_Init(void);
#endif

