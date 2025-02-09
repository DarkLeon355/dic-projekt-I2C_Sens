#include "project.h"
#include <stdio.h>

#define BMP180_ADDR 0x77  // BMP180 I2C addresse

// kalibrations variablen
int16 AC1, AC2, AC3, B1, B2, MB, MC, MD;
uint16 AC4, AC5, AC6;




void BMP180_WriteByte(uint8 reg, uint8 value)
{
    uint8 data[2] = { reg, value };
    I2C_MasterWriteBuf(BMP180_ADDR, data, 2, I2C_MODE_COMPLETE_XFER);
    while (I2C_MasterStatus() & I2C_MSTAT_XFER_INP);
}

uint16 BMP180_ReadWord(uint8 reg)
{
    uint8 data[2];
    I2C_MasterWriteBuf(BMP180_ADDR, &reg, 1, I2C_MODE_COMPLETE_XFER);
    while (I2C_MasterStatus() & I2C_MSTAT_XFER_INP);

    I2C_MasterReadBuf(BMP180_ADDR, data, 2, I2C_MODE_COMPLETE_XFER);
    while (I2C_MasterStatus() & I2C_MSTAT_XFER_INP);

    return ((uint16)data[0] << 8) | data[1];
}


void BMP180_ReadCalibrationData(void)
{
    AC1 = (int16)BMP180_ReadWord(0xAA);
    AC2 = (int16)BMP180_ReadWord(0xAC);
    AC3 = (int16)BMP180_ReadWord(0xAE);
    AC4 = BMP180_ReadWord(0xB0);
    AC5 = BMP180_ReadWord(0xB2);
    AC6 = BMP180_ReadWord(0xB4);
    B1  = (int16)BMP180_ReadWord(0xB6);
    B2  = (int16)BMP180_ReadWord(0xB8);
    MB  = (int16)BMP180_ReadWord(0xBA);
    MC  = (int16)BMP180_ReadWord(0xBC);
    MD  = (int16)BMP180_ReadWord(0xBE);
}

int16 BMP180_ReadRawTemperature(void)
{
    BMP180_WriteByte(0xF4, 0x2E);
    CyDelay(5); 
    return BMP180_ReadWord(0xF6);
}

int32 BMP180_ReadRawPressure(void)
{
    BMP180_WriteByte(0xF4, 0x34);
    CyDelay(8);
    return (int32)BMP180_ReadWord(0xF6);
}


float BMP180_CalculateTemperature(int16 ut, int32 *B5)
{
    int32 X1 = (((int32)ut - AC6) * AC5) >> 15;
    int32 X2 = ((int32)MC << 11) / (X1 + MD);
    *B5 = X1 + X2;
    float T = (((*B5 + 8) >> 4)) / 10.0;  // Temperature in °C
    return T;
}

int32 BMP180_CalculatePressure(int32 up, int32 B5)
{
    int32 B6 = B5 - 4000;
    int32 X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
    int32 X2 = (AC2 * B6) >> 11;
    int32 X3 = X1 + X2;
    int32 B3 = (((((int32)AC1) * 4 + X3) + 2) / 4);
    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32 B4 = (AC4 * (uint32)(X3 + 32768)) >> 15;
    uint32 B7 = ((uint32)up - B3) * 50000;
    int32 P;
    if (B7 < 0x80000000)
    {
        P = (B7 << 1) / B4;
    }
    else
    {
        P = (B7 / B4) << 1;
    }
    X1 = (P >> 8) * (P >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * P) >> 16;
    P = P + ((X1 + X2 + 3791) >> 4);
    return P;
}


void BMP180_Init(void)
{
    I2C_Start();
    UART_Start();
    BMP180_ReadCalibrationData();

}

void UART_Print(const char *string)
{
    UART_PutString(string);
}

int main(void)
{
    CyGlobalIntEnable;
    char buffer[50];

    BMP180_Init();

    for (;;)
    {
        int16 ut = BMP180_ReadRawTemperature();
        int32 up = BMP180_ReadRawPressure();
        int32 B5;
        float temperature = BMP180_CalculateTemperature(ut, &B5);
        int32 pressure = BMP180_CalculatePressure(up, B5);
        int16 temp_int = (int16)temperature;
        int16 temp_frac = (int16)((temperature - temp_int) * 100);
        sprintf(buffer, "Temperature: %d.%02d C\r\n", temp_int, (temp_frac < 0 ? -temp_frac : temp_frac));
        UART_Print(buffer);

        sprintf(buffer, "Pressure: %ld Pa\r\n", pressure);
        UART_Print(buffer);

        CyDelay(2000);
    }
}
