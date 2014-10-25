#include <stdlib.h> 
#include "stm32f4xx_conf.h" 
#include "i2s_setup.h"

int main(void)
{
    i2s_full_duplex_setup();
    while (1);

}
