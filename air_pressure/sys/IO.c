#include "IO.h"
#include "delay.h"
#include "TIM.h"
#include "math.h"
#include "stdlib.h"
// 2�� 1ͨ�� 1����  1��ˮ 1��ˮ 1��ʪ 2��IIC ��ʪ�� ���� 2��ADC  ˮλ ����ʪ��
#if 1//IIC
void IIC_Init(void)
{
	IIC_SCL_OUT;
	IIC_SDA_OUT;
}
void IIC_Start(void)//����IIC��ʼ�ź�
{
	IIC_SDA_OUT;     //sda�����
	IIC_SDA_SET;//IIC_SDA=1;	  	  
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SDA_RESET;//IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}
void IIC_Stop(void)//����IICֹͣ�ź�
{
	IIC_SDA_OUT;//sda�����
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_RESET;//IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1; 
	delay_us(5);
	IIC_SDA_SET;//IIC_SDA=1;//����I2C���߽����ź�				   	
}
void IIC_Ack(void)//����ACKӦ��
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_RESET;//IIC_SDA=0;
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}   
void IIC_NAck(void)//������ACKӦ��	
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_SET;//IIC_SDA=1;
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}	
uint8_t IIC_Wait_Ack(void)//�ȴ�Ӧ���źŵ���:1,����Ӧ��ʧ��;0,����Ӧ��ɹ�
{
	uint8_t ucErrTime=0;
	IIC_SDA_IN;      //SDA����Ϊ����     
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);	 
	while(IIC_SDA_State)//���SDA�Ƿ���Ϊ�ߵ�ƽ
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_RESET;//IIC_SCL=0;
	return 0;  
} 
void IIC_Send_Byte(uint8_t txd)//IIC����һ���ֽ�; �ȷ��͸�λ
{                        
	uint8_t t;   
	IIC_SDA_OUT; 	    
	IIC_SCL_RESET;//IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{              
			//IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA_SET;//IIC_SDA=1;
		else
			IIC_SDA_RESET;//IIC_SDA=0;
		
		txd<<=1; 	  
		delay_us(5);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL_SET;//IIC_SCL=1;
		delay_us(5); 
		IIC_SCL_RESET;//IIC_SCL=0;	
		delay_us(5);
	}	 
} 
uint8_t IIC_Read_Byte(unsigned char ack)//��һ���ֽڣ��ɼ��Ƿ�Ӧ��λ,1��ack��0����ack �Ӹ�λ��ʼ��
{
	unsigned char i,receive=0;
	IIC_SDA_IN;//SDA����Ϊ����
	for(i=0;i<8;i++ )
	{
		IIC_SCL_RESET;// IIC_SCL=0; 
		delay_us(5);
		IIC_SCL_SET;//IIC_SCL=1;
		receive<<=1;
		if(IIC_SDA_State)
			receive++;   
		delay_us(5); 
	}					 
		if (ack)
			IIC_Ack(); //����ACK
		else
			IIC_NAck();//����nACK   
		return receive;
}

uint8_t IIC_Write_Byte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t data)//ֱ��дһ���ֽ�
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//���͵�ַ	  
	ret |= IIC_Wait_Ack();		
	IIC_Send_Byte(data);     //�����ֽ�							   
	ret |= IIC_Wait_Ack(); 

	IIC_Stop();
	delay_us(10);
	return ret;
}
uint8_t IIC_ReadMulByte(uint8_t DrvAddr,uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)//���ֽ�
{  	    																 
	uint8_t ret=0;
	
	IIC_Start();  
	IIC_Send_Byte(DrvAddr);	   //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr);//���͵�ַ	    
	ret |= IIC_Wait_Ack();	    
	
	IIC_Start();
	IIC_Send_Byte(DrvAddr+1);           //�������ģʽ			   
	ret |= IIC_Wait_Ack();
	while(NumToRead)
	{
		if(NumToRead==1)
		{
			*pBuffer=IIC_Read_Byte(0);	
		}
		else
		{
			*pBuffer=IIC_Read_Byte(1);
		}
		pBuffer++;
		NumToRead--;
	}
	IIC_Stop();//����һ��ֹͣ����	
	return ret;	
}
uint8_t IIC_WriteMulByte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)//��һ��д����ֽ�
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//���͵�ַ	  
	ret |= IIC_Wait_Ack();		

	while(NumToWrite--)
	{ 										  		   
		IIC_Send_Byte(*pBuffer);     //�����ֽ�							   
		ret |= IIC_Wait_Ack(); 
		pBuffer++;
	}
	IIC_Stop();
	delay_us(10);
	return ret;
}
#endif

/******************************************************�¶���ѹ******************************************************/
uint8_t F_TASK_BMP180=0;
struct ctrl_state gas_temp;
struct ctrl_state gas_pressure;
#if USING_BMP180//�¶���ѹ
struct bmp180_calib_param_t calib_param;
uint8_t BMP180_param[BMP180_PROM_DATA__LEN];
int32_t param_b5;
uint8_t oversamp_setting=0;//����Ϊ0~3
void bmp180_get_calib_param(void)
{
  IIC_ReadMulByte(BMP180_I2C_ADDR,BMP180_PROM_START__ADDR,BMP180_param,BMP180_PROM_DATA__LEN);
	
    /*parameters AC1-AC6*/
    calib_param.ac1 =
        (int16_t)((((int32_t)((int8_t)BMP180_param[BMP180_CALIB_PARAM_AC1_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_AC1_LSB]);
    calib_param.ac2 =
        (int16_t)((((int32_t)((int8_t)BMP180_param[BMP180_CALIB_PARAM_AC2_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_AC2_LSB]);
    calib_param.ac3 =
        (int16_t)((((int32_t)((int8_t)BMP180_param[BMP180_CALIB_PARAM_AC3_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_AC3_LSB]);
    calib_param.ac4 =
        (uint16_t)((((uint32_t)((uint8_t)BMP180_param[BMP180_CALIB_PARAM_AC4_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_AC4_LSB]);
    calib_param.ac5 =
        (uint16_t)((((uint32_t)((uint8_t)BMP180_param[BMP180_CALIB_PARAM_AC5_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_AC5_LSB]);
    calib_param.ac6 =
        (uint16_t)((((uint32_t)((uint8_t)BMP180_param[BMP180_CALIB_PARAM_AC6_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_AC6_LSB]);

    /*parameters B1,B2*/
    calib_param.b1 =
        (int16_t)((((int32_t)((int8_t)BMP180_param[BMP180_CALIB_PARAM_B1_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_B1_LSB]);
    calib_param.b2 =
        (int16_t)((((int32_t)((int8_t)BMP180_param[BMP180_CALIB_PARAM_B2_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_B2_LSB]);

    /*parameters MB,MC,MD*/
    calib_param.mb =
        (int16_t)((((int32_t)((int8_t)BMP180_param[BMP180_CALIB_PARAM_MB_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_MB_LSB]);
    calib_param.mc =
        (int16_t)((((int32_t)((int8_t)BMP180_param[BMP180_CALIB_PARAM_MC_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_MC_LSB]);
    calib_param.md =
        (int16_t)((((int32_t)((int8_t)BMP180_param[BMP180_CALIB_PARAM_MD_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_MD_LSB]);	
}
void BMP180_Init(void)
{
	uint8_t ID;
	IIC_ReadMulByte(BMP180_I2C_ADDR,BMP180_CHIP_ID_REG,&ID,1);
	if(ID==0X55)
	{
		oversamp_setting=1;//����Ϊ0~3
		bmp180_get_calib_param();
	}
}
uint16_t bmp180_get_uncomp_temperature(void)
{
    uint16_t v_ut_u16 = BMP180_INIT_VALUE;
    uint8_t error=0;
    /* Array holding the temperature LSB and MSB data*/
    uint8_t v_data_u8[BMP180_TEMPERATURE_DATA_BYTES] = { 0, 0 };
    uint8_t v_ctrl_reg_data_u8 = BMP180_T_MEASURE;
    
    /* used to return the bus communication results*/
    
    error += IIC_WriteMulByte(BMP180_I2C_ADDR,BMP180_CTRL_MEAS_REG,&v_ctrl_reg_data_u8,BMP180_GEN_READ_WRITE_DATA_LENGTH);
		delay_ms(5);
		error += IIC_ReadMulByte(BMP180_I2C_ADDR,BMP180_ADC_OUT_MSB_REG,v_data_u8,BMP180_TEMPERATURE_DATA_LENGTH);
		
    if (error == 0)
    {
        v_ut_u16 =
            (uint16_t)((((int32_t)((int8_t)v_data_u8[BMP180_TEMPERATURE_MSB_DATA])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
                  (v_data_u8[BMP180_TEMPERATURE_LSB_DATA]));
    }
    return v_ut_u16;
}
int16_t bmp180_get_temperature(uint32_t v_uncomp_temperature_u32)
{
    int16_t v_temperature_s16 = BMP180_INIT_VALUE;
    int32_t v_x1_s32, v_x2_s32 = BMP180_INIT_VALUE;

    /* calculate temperature*/
    v_x1_s32 = (((int32_t) v_uncomp_temperature_u32 - (int32_t) calib_param.ac6) * (int32_t) calib_param.ac5) >>
               BMP180_SHIFT_BIT_POSITION_BY_15_BITS;
    if (v_x1_s32 == BMP180_CHECK_DIVISOR && calib_param.md == BMP180_CHECK_DIVISOR)
    {
        return BMP180_INVALID_DATA;
    }

    /* executed only the divisor is not zero*/
    v_x2_s32 = ((int32_t) calib_param.mc << BMP180_SHIFT_BIT_POSITION_BY_11_BITS) /
               (v_x1_s32 + calib_param.md);
    param_b5 = v_x1_s32 + v_x2_s32;
    v_temperature_s16 =
        ((param_b5 + BMP180_CALCULATE_TRUE_TEMPERATURE) >> BMP180_SHIFT_BIT_POSITION_BY_04_BITS);

    return v_temperature_s16;
}

uint32_t bmp180_get_uncomp_pressure(uint8_t oss_mode)
{
	/*j included for loop*/
	//uint8_t v_j_u8 = BMP180_INIT_VALUE;
	uint32_t v_up_u32 = BMP180_INIT_VALUE;
	
	/*get the calculated pressure data*/
	//uint32_t v_sum_u32 = BMP180_INIT_VALUE;
	uint8_t v_data_u8[BMP180_PRESSURE_DATA_BYTES] = { BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE };
	uint8_t v_ctrl_reg_data_u8 = BMP180_INIT_VALUE;

	/* used to return the bus communication results*/
  uint8_t error=0;

	v_ctrl_reg_data_u8 = BMP180_P_MEASURE + ( oss_mode << BMP180_SHIFT_BIT_POSITION_BY_06_BITS);
	error += IIC_WriteMulByte(BMP180_I2C_ADDR,BMP180_CTRL_MEAS_REG,&v_ctrl_reg_data_u8,BMP180_GEN_READ_WRITE_DATA_LENGTH);
	delay_ms(BMP180_2MS_DELAY_U8X + (BMP180_3MS_DELAY_U8X << (oss_mode)));
	error += IIC_ReadMulByte(BMP180_I2C_ADDR,BMP180_ADC_OUT_MSB_REG,v_data_u8,BMP180_PRESSURE_DATA_LENGTH);
	if (error == 0)
	{
			v_up_u32 =
					(uint32_t)((((uint32_t)v_data_u8[BMP180_PRESSURE_MSB_DATA] << BMP180_SHIFT_BIT_POSITION_BY_16_BITS) |
								 ((uint32_t) v_data_u8[BMP180_PRESSURE_LSB_DATA] << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
								 (uint32_t) v_data_u8[BMP180_PRESSURE_XLSB_DATA]) >>
								(BMP180_CALCULATE_TRUE_PRESSURE - oss_mode));
	}
	return v_up_u32;
}
int32_t bmp180_get_pressure(uint32_t v_uncomp_pressure_u32)
{
	int32_t v_pressure_s32, v_x1_s32, v_x2_s32, v_x3_s32, v_b3_s32, v_b6_s32 = BMP180_INIT_VALUE;
	uint32_t v_b4_u32, v_b7_u32 = BMP180_INIT_VALUE;

	v_b6_s32 = param_b5 - 4000;

	/*****calculate B3************/
	v_x1_s32 = (v_b6_s32 * v_b6_s32) >> BMP180_SHIFT_BIT_POSITION_BY_12_BITS;
	v_x1_s32 *= calib_param.b2;
	v_x1_s32 >>= BMP180_SHIFT_BIT_POSITION_BY_11_BITS;
	v_x2_s32 = (calib_param.ac2 * v_b6_s32);
	v_x2_s32 >>= BMP180_SHIFT_BIT_POSITION_BY_11_BITS;
	v_x3_s32 = v_x1_s32 + v_x2_s32;
	v_b3_s32 = (((((int32_t)calib_param.ac1) * 4 + v_x3_s32) << oversamp_setting) + 2) >>
	BMP180_SHIFT_BIT_POSITION_BY_02_BITS;

	/*****calculate B4************/
	v_x1_s32 = (calib_param.ac3 * v_b6_s32) >> BMP180_SHIFT_BIT_POSITION_BY_13_BITS;
	v_x2_s32 = (calib_param.b1 * ((v_b6_s32 * v_b6_s32) >> BMP180_SHIFT_BIT_POSITION_BY_12_BITS)) >>
	BMP180_SHIFT_BIT_POSITION_BY_16_BITS;
	v_x3_s32 = ((v_x1_s32 + v_x2_s32) + 2) >> BMP180_SHIFT_BIT_POSITION_BY_02_BITS;
	v_b4_u32 = (calib_param.ac4 * (uint32_t)(v_x3_s32 + 32768)) >> BMP180_SHIFT_BIT_POSITION_BY_15_BITS;
	v_b7_u32 = ((uint32_t)(v_uncomp_pressure_u32 - v_b3_s32) * (50000 >> oversamp_setting));
	if (v_b7_u32 < 0x80000000)
	{
		if (v_b4_u32 != BMP180_CHECK_DIVISOR)
		{
			v_pressure_s32 = (v_b7_u32 << BMP180_SHIFT_BIT_POSITION_BY_01_BIT) / v_b4_u32;
		}
		else
		{
			return BMP180_INVALID_DATA;
		}
	}
	else
	{
		if (v_b4_u32 != BMP180_CHECK_DIVISOR)
		{
			v_pressure_s32 = (v_b7_u32 / v_b4_u32) << BMP180_SHIFT_BIT_POSITION_BY_01_BIT;
		}
		else
		{
			return BMP180_INVALID_DATA;
		}
	}
	v_x1_s32 = v_pressure_s32 >> BMP180_SHIFT_BIT_POSITION_BY_08_BITS;
	v_x1_s32 *= v_x1_s32;
	v_x1_s32 = (v_x1_s32 * BMP180_PARAM_MG) >> BMP180_SHIFT_BIT_POSITION_BY_16_BITS;
	v_x2_s32 = (v_pressure_s32 * BMP180_PARAM_MH) >> BMP180_SHIFT_BIT_POSITION_BY_16_BITS;

	/*pressure in Pa*/
	v_pressure_s32 += (v_x1_s32 + v_x2_s32 + BMP180_PARAM_MI) >> BMP180_SHIFT_BIT_POSITION_BY_04_BITS;

	return v_pressure_s32;
}
void TASK_BMP180(void)
{
	uint16_t v_uncomp_temp_u16 = 0;
	uint32_t v_uncomp_press_u32 = 0;

	v_uncomp_temp_u16 = bmp180_get_uncomp_temperature();
	gas_temp.now=bmp180_get_temperature(v_uncomp_temp_u16);
  mcu_dp_value_update(DPID_TEMPERATURE_NOW,(uint32_t)gas_temp.now);
	
	v_uncomp_press_u32=bmp180_get_uncomp_pressure(oversamp_setting);
	gas_pressure.now=bmp180_get_pressure(v_uncomp_press_u32);
  mcu_dp_value_update(DPID_PRESSURE_NOW,(uint32_t)gas_pressure.now);
	
}
#endif
void Modules_Init(void)
{
	IIC_Init();
	BMP180_Init();
}
void SwitchIO_Init(void)
{
	LED_4_OUT;	
}
void IO_Init(void)
{
	Modules_Init();
	SwitchIO_Init();
}
