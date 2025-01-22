#include "project.h"

int main(void)
{
    CyGlobalIntEnable; 
    
    uint8 rbuf[20];
    uint8 SlaveAddr = 0x77;
    uint8 wrData,cnt = 0x0;
    uint8 rdData = 0;
    uint8 mode = 0;
    
    I2C_Master_MasterWriteBuf(SlaveAddr, &wrData, cnt, mode);
    
    I2C_Master_Start();

    for(;;)
    {
        uint8 I2C_Master_MasterReadBuf(uint8 SlaveAddr,
            uint8 * rdData, uint8 cnt, uint8 mode);
        
        if (&rdData > 0)
        {
            Control_Reg_1_Write(255);
            CyDelay(1000);
            Control_Reg_1_Write(0);
            CyDelay(1000);
        }
        else
        {
            Control_Reg_1_Write(0);
        }
        
    }
}


