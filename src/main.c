#include <stdlib.h> 
#include <stdint.h> 
#include <math.h> 
#include "stm32f4xx_conf.h" 
#include "i2s_setup.h"

#define PHASE_INC 1. / 44100. * 4. * 2. * M_PI 

int main(void)
{
    size_t i;
//    i2s_full_duplex_setup();
    i2s_dma_full_duplex_setup();
    float phase;
    while (1) {
        /* wait until buffers are available */
        while (!(codecDmaRxPtr && codecDmaTxPtr));
        /* do stuff with data */
        /* here we just copy input to output */
        for (i = 0; i < CODEC_DMA_BUF_LEN; i += 2) {

            codecDmaTxPtr[i] = FLOAT_TO_INT16(
                    INT16_TO_FLOAT(codecDmaRxPtr[i]) * (sinf(phase) + 1.) / 2.);
            codecDmaTxPtr[i+1] = FLOAT_TO_INT16(
                    INT16_TO_FLOAT(codecDmaRxPtr[i+1]) * (sinf(phase) + 1.) / 2.);
            phase += PHASE_INC;
            while (phase >= (2. * M_PI)) {
                phase -= 2. * M_PI;
            }
        }
        /* don't forget to set the pointers to NULL after you're done with them
         * */
        codecDmaTxPtr = NULL;
        codecDmaRxPtr = NULL;
    } 
}
