#include <stm32f1xx.h>
#include <stm32f1xx_hal.h>

#include "pinout.h"
#include "config.h"


int main(void)
{
    HAL_Init();

    while (1)
    {

    }
}


// overwrite printf() output to send via ITM (SWO)
int _write(int file, char *data, int len)
{
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(data[i]);
    }

    // return # of bytes written - as best we can tell
    return len;
}
