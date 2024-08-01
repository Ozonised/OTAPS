#include <stm32f1xx.h>
#include <stm32f1xx_hal.h>

#include "nodes.h"


int main(void)
{
    HAL_Init();
    nodeInitialisation();
    while (1)
    {
        postInit();
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
