#include "include.h"
#ifdef LQ

void Init2100()
{
        I2C_WriteAddr(I2C0,SlaveAddress2100,0x0d,0x02);

        Pause();

        I2C_WriteAddr(I2C0,SlaveAddress2100,CTRL_REG1_2100,0x02);

        Pause();
}

void Init8700()
{

     //  I2C_WriteAddr(I2C1,SlaveAddress8700,0x0f,0x33);

     //   Pause();

        I2C_WriteAddr(I2C0,SlaveAddress8700,CTRL_REG1_8700,0x05);

        Pause();
}




int16 read_lq(uint8 slave,uint8 Axis) 
{
    uint8 bit8_data[2]={0};
    int16 s=0;
    int16 MUP_Zero=0x0000;
    switch(Axis)
    {
        case 'X':
            bit8_data[0]=I2C_ReadAddr(I2C0,slave,OUT_X_MSB_REG);
            nops();
            nops();
            nops();
            bit8_data[1]=I2C_ReadAddr(I2C0,slave,OUT_X_LSB_REG);
            nops();
            nops();
            nops();
            break;
        case 'Y':
            bit8_data[0]=I2C_ReadAddr(I2C0,slave,OUT_Y_MSB_REG);
            nops();
            nops();
            nops();
            bit8_data[1]=I2C_ReadAddr(I2C0,slave,OUT_Y_LSB_REG);
            nops();
            nops();
            nops();
            break;
        case 'Z':
            bit8_data[0]=I2C_ReadAddr(I2C0,slave,OUT_Z_MSB_REG);
            nops();
            nops();
            nops();
            bit8_data[1]=I2C_ReadAddr(I2C0,slave,OUT_Z_LSB_REG);
            nops();
            nops();
            nops();
            break;
        default:break;
    }
    s = (((MUP_Zero | bit8_data[0])<<8)|bit8_data[1]);
    return s;
    
}


#endif

