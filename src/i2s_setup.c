#include "stm32f4xx.h"
#include <math.h>

#define UINT16_TO_FLOAT(x) ((float)(x - 0x7fff)/((float)0x7fff))
#define FLOAT_TO_UINT16(x) ((uint16_t)((x + 1.) * 0x7fff))

void i2s_setup(void)
{
    /* Turn on GPIO clock for I2S3 pins */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    /* Configure GPIO */
    /* Configure PA15 to Alternate Function */
    GPIOA->MODER &= ~(0x3 << 30);
    GPIOA->MODER |= (0x2 << 30);
    /* Configure PC7, PC10-12 to Alternate Function */
    GPIOC->MODER &= ~((0x3 << 20) | (0x3 << 22) | (0x3 << 24) | (0x3 << 14));
    GPIOC->MODER |= (0x2 << 20) | (0x2 << 22) | (0x2 << 24) | (0x2 << 14);
    /* Set pins to high speed */
    GPIOA->OSPEEDR |= (0x3 << 30);
    GPIOC->OSPEEDR |= (0x3 << 20) | (0x3 << 22) | (0x3 << 24) | (0x3 << 14);
    /* Pins have no-pull up nor pull-down */
    GPIOA->MODER &= ~(0x3 << 30);
    GPIOC->MODER &= ~((0x3 << 20) | (0x3 << 22) | (0x3 << 24) | (0x3 << 14));
    /* A15 Alternate function 6 */
    GPIOA->AFR[1] &= ~(0xf << 28);
    GPIOA->AFR[1] |= (0x6 << 28);
    /* C7 Alternate function 6 */
    GPIOC->AFR[0] &= ~(0xf << 28);
    GPIOC->AFR[0] |= (0x6 << 28);
    /* C10,12, Alternate function 6, C11 alternate function 5 */
    GPIOC->AFR[1] &= ~((0xf << 8) | (0xf << 12) | (0xf << 16));
    GPIOC->AFR[1] |= ((0x6 << 8) | (0x5 << 12) | (0x6 << 16));

    /* Turn on I2S3 clock (SPI3) */
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    /* PLLI2S_R and PLLI2S_N have been setup in system_stm32f4xx.c */
    /* I2SDIV = 6, MCK on, ODD is set to 0 */
    SPI3->I2SPR = ((0x2 << 8) | 0x6);
    /* CKPOL = 0, I2SMOD = 1, I2SEN = 0 (don't enable yet), I2SSTD = 00
     * (Phillips), DATLEN = 00 (16-bit), CHLEN = 0 (16-bit) I2SCFGR = 10 (Master
     * transmit) */
    SPI3->I2SCFGR = 0xe00;
    /* TXEIE = 1 (Transmit buffer empty interrupt enable), other bits off */
    SPI3->CR2 = 0x80;

    /* Enable I2S interrupt */
    NVIC_EnableIRQ(SPI3_IRQn);
}

void i2s_full_duplex_setup(void)
{
    /* Turn on GPIO clock for I2S3 pins */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    /* Configure GPIO */
    /* Configure PA15 to Alternate Function */
    GPIOA->MODER &= ~(0x3 << 30);
    GPIOA->MODER |= (0x2 << 30);
    /* Configure PC7, PC10-12 to Alternate Function */
    GPIOC->MODER &= ~((0x3 << 20) | (0x3 << 22) | (0x3 << 24) | (0x3 << 14));
    GPIOC->MODER |= (0x2 << 20) | (0x2 << 22) | (0x2 << 24) | (0x2 << 14);
    /* Set pins to high speed */
    GPIOA->OSPEEDR |= (0x3 << 30);
    GPIOC->OSPEEDR |= (0x3 << 20) | (0x3 << 22) | (0x3 << 24) | (0x3 << 14);
    /* Pins have no-pull up nor pull-down */
    GPIOA->PUPDR &= ~(0x3 << 30);
    GPIOC->PUPDR &= ~((0x3 << 20) | (0x3 << 22) | (0x3 << 24) | (0x3 << 14));
    /* A15 Alternate function 6 */
    GPIOA->AFR[1] &= ~(0xf << 28);
    GPIOA->AFR[1] |= (0x6 << 28);
    /* C7 Alternate function 6 */
    GPIOC->AFR[0] &= ~(0xf << 28);
    GPIOC->AFR[0] |= (0x6 << 28);
    /* C10,12, Alternate function 6, C11 alternate function 5 */
    GPIOC->AFR[1] &= ~((0xf << 8) | (0xf << 12) | (0xf << 16));
    GPIOC->AFR[1] |= ((0x6 << 8) | (0x5 << 12) | (0x6 << 16));

    /* Turn on I2S3 clock (SPI3) */
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    /* PLLI2S_R and PLLI2S_N have been setup in system_stm32f4xx.c */
    /* I2SDIV = 6, MCK on, ODD is set to 0 */
    SPI3->I2SPR = ((0x2 << 8) | 0x6);
//    SPI3->I2SPR = ((0x3 << 8) | 0xc);
    /* CKPOL = 0, I2SMOD = 1, I2SEN = 0 (don't enable yet), I2SSTD = 00
     * (Phillips), DATLEN = 00 (16-bit), CHLEN = 0 (16-bit) I2SCFGR = 10 (Master
     * transmit) */
    SPI3->I2SCFGR = 0xa00;
    /* TXEIE = 1 (Transmit buffer empty interrupt enable), other bits off */
    SPI3->CR2 = 0x80;
    /* Set up duplex instance the same as SPI3, except configure as slave
     * receive and trigger interrupt when receive buffer full */
//    I2S3ext->I2SPR = ((0x3 << 8) | 0xc);
    I2S3ext->I2SPR = ((0x2 << 8) | 0x6);
    /* same as above but I2SCFG = 01 (slave receive) */
    I2S3ext->I2SCFGR = 0x900;
    /* RXNEIE = 1 (Receive buffer not empty interrupt enable), other bits off */
    I2S3ext->CR2 = 0x40;

    /* Enable I2S interrupt */
    NVIC_EnableIRQ(SPI3_IRQn);

    /* Turn on I2S3 and its extended block */
    I2S3ext->I2SCFGR |= 0x400;
    SPI3->I2SCFGR |= 0x400;
}

void SPI3_IRQHandler(void)
{
#define CTRMAX 44
    static uint16_t lastval;
    static uint16_t lctr, lval;
    static uint16_t rctr, rval;
    float lphase = 0;
    float rphase = 0;
#define PHASE_INC (0.25 / 44100.0 * 2. * M_PI)
    NVIC_ClearPendingIRQ(SPI3_IRQn);
    if (I2S3ext->SR & SPI_SR_RXNE); {
        if (I2S3ext->SR & SPI_SR_CHSIDE) {
            rval = I2S3ext->DR;
        } else {
            lval = I2S3ext->DR;
        }
/*         lastval = I2S3ext->DR; */
        I2S3ext->SR &= ~(SPI_SR_RXNE);
    }
    if (SPI3->SR & SPI_SR_TXE); {
        /*
        if (SPI3->SR & SPI_SR_CHSIDE) {
            SPI3->DR = lval * 0x7fff;
            if (lctr++ > CTRMAX) {
                lval = 1 - lval;
                lctr = 0;
            }
        } else {
            SPI3->DR = rval * 0x7fff;
            if (rctr++ > CTRMAX) {
                rval = 1 - rval;
                rctr = 0;
            }
        }
        */
        if (SPI3->SR & SPI_SR_CHSIDE) {
            SPI3->DR = FLOAT_TO_UINT16(UINT16_TO_FLOAT(rval));
//            SPI3->DR = FLOAT_TO_UINT16(UINT16_TO_FLOAT(rval) * (sinf(rphase) + 1.) / 2.);
            rphase += PHASE_INC;
//            while (rphase >= 2. * M_PI) {
//                rphase -= 2. * M_PI;
//            }
        } else {
            SPI3->DR = FLOAT_TO_UINT16(UINT16_TO_FLOAT(lval));
//            SPI3->DR = FLOAT_TO_UINT16(UINT16_TO_FLOAT(lval) * (sinf(lphase) + 1.) / 2.);
            lphase += PHASE_INC;
//            while (lphase >= 2. * M_PI) {
//                lphase -= 2. * M_PI;
//            }
        }
        /*
        SPI3->DR = 0x0f;
        SPI3->DR = lastval;
        */
        SPI3->SR &= ~(SPI_SR_TXE);
    }
}
