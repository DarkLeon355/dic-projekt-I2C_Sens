#include "project.h"

int main(void)
{
    CyGlobalIntEnable; 
    uint8 reg = 0x76;
    uint8 rbuf[20];
    
    I2C_Master_MasterReadBuf((uint8 *) rbuf, 20);

    
    I2C_Master_Start();

    for(;;)
    {
        /* Place your application code here. */
    }
}


