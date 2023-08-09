#include "stm32f4xx.h"

/* Configuration */

#define USART_REG       USART1
#define USART_TX_BIT    9
#define USART_RX_BIT    10
#define USART_IRQn      USART1_IRQn

#define UART_BASECLK   84000000

uint8_t tx_finish = 0;
uint8_t rx_finish = 0;

/* Initialize USART TX/RX with DMA
 * TX: normal mode
 * RX: circular mode
 * Note:
 * Transmitter in circular mode is difficult to use because
 * it starts next transmission immediately.
 * You cannot modify correctly unless using double buffer mode
 */
void USART_DMA_init(uint32_t baudrate, char *rx_buffer)
{
        uint32_t v, div;

        /* Enable PORTA and DMA2 clock */
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;

        /* Enable USART1 clock */
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

        /*   Initialize TX DMA (Stream 7, CH4)   */

        /* Wait DMA stream to disable if current tranfers are not finished */
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        while (DMA2_Stream7->CR & DMA_SxCR_EN);
        DMA2_Stream7->CR = 0;
        /* Set peripheral address */
        DMA2_Stream7->PAR = (uint32_t)&USART1->DR;

        /* Set DMA configuration */
        v = DMA_CHANNEL_4
          | DMA_MBURST_SINGLE   /* Memory burst single */
          | DMA_PBURST_SINGLE   /* Peripheral burst single */
          | DMA_PRIORITY_LOW
          | 0                   /* Peripheral increment offset: by PSIZE */
          | DMA_MDATAALIGN_BYTE /* Mememory data size */
          | DMA_PDATAALIGN_BYTE /* Peripheral data size */
          | DMA_MINC_ENABLE     /* Memory increment mode enable */
          | DMA_PINC_DISABLE    /* Peripheral increment mode disble */
          | DMA_NORMAL          /* Circular mode: disable */
          | DMA_MEMORY_TO_PERIPH /* Data transfer direction */
          | 0                   /* Flow controller: DMA */
          | DMA_SxCR_TCIE;      /* Transfer complete interrupt enable */
        DMA2_Stream7->CR |= v;

        /* Setup callbacks which are called by ISR handler and enable
         * interrupt in NVIC (priority: 9) */
        NVIC_SetPriority(DMA2_Stream7_IRQn, 9);
        NVIC_EnableIRQ(DMA2_Stream7_IRQn);

        /*   Initialize RX DMA (Stream 5, CH4)   */

        /* Wait DMA stream to disable if current tranfers are not finished */
        DMA2_Stream5->CR &= ~DMA_SxCR_EN;
        while (DMA2_Stream5->CR & DMA_SxCR_EN);
        DMA2_Stream5->CR = 0;
        /* Set peripheral address */
        DMA2_Stream5->PAR = (uint32_t)&USART1->DR;

        /* Set memory address */
        DMA2_Stream5->M0AR = (uint32_t)rx_buffer;

        /* Set the total number of data items to be transferred */
        DMA2_Stream5->NDTR = 1U;

        /* Set DMA configuration */
        v = DMA_CHANNEL_4
          | DMA_MBURST_SINGLE   /* Memory burst single */
          | DMA_PBURST_SINGLE   /* Peripheral burst single */
          | DMA_PRIORITY_VERY_HIGH
          | 0                   /* Peripheral increment offset: by PSIZE */
          | DMA_MDATAALIGN_BYTE /* Mememory data size */
          | DMA_PDATAALIGN_BYTE /* Peripheral data size */
          | DMA_MINC_DISABLE    /* Memory increment mode disable */
          | DMA_PINC_DISABLE    /* Peripheral increment mode disble */
          | DMA_CIRCULAR        /* Circular mode: disable */
          | DMA_PERIPH_TO_MEMORY /* Data transfer direction */
          | 0                   /* Flow controller: DMA */
          | DMA_SxCR_TCIE;      /* Transfer complete interrupt enable */
        DMA2_Stream5->CR |= v;

        /* Setup callbacks which are called by ISR handler and enable
         * interrupt in NVIC (priority: 6) */
        NVIC_SetPriority(DMA2_Stream5_IRQn, 6);
        NVIC_EnableIRQ(DMA2_Stream5_IRQn);

        /* Configure USART alternate function */
        v  = GPIOA->AFR[USART_TX_BIT >> 3];
        v &= ~(15UL << ((USART_TX_BIT & 0x7) << 2));
        v |=   (7UL << ((USART_TX_BIT & 0x7) << 2));
        GPIOA->AFR[USART_TX_BIT >> 3] = v;
        v  = GPIOA->AFR[USART_RX_BIT >> 3];
        v &= ~(15UL << ((USART_RX_BIT & 0x7) << 2));
        v |=   (7UL << ((USART_RX_BIT & 0x7) << 2));
        GPIOA->AFR[USART_RX_BIT >> 3] = v;

        /* Configure USART RX/TX pins for alternate function usage */
        v  = GPIOA->MODER;
        v &= ~((3UL << (USART_TX_BIT << 1)) | (3UL << (USART_RX_BIT << 1)));
        v |=  ((2UL << (USART_TX_BIT << 1)) | (2UL << (USART_RX_BIT << 1)));         // PA10: alternate function
        GPIOA->MODER = v;

        /* Initialize USART */
        v = USART_CR1_OVER8     /* oversampline by 8 */
          | 0                   /* USART disable */
          | 0                   /* 1 Start, 8 Data, n Stop bits */
          | 0                   /* Parity control disable */
          | 0                   /* TX empty interrupt disable */
          | 0                   /* TX complete interrupt disable */
          | 0                   /* RX not empty interrupt disable */
          | USART_CR1_TE        /* Transmitter enable */
          | USART_CR1_RE;       /* Receiver enable */
        USART_REG->CR1 = v;

        USART_REG->CR2 = 0;     /* STOP bits: 1 Stop bit */
        
        /* Set baudrate */
        div = (UART_BASECLK << 1) / baudrate;
        div += (div & 0x1) << 1; /* rounding */
        div = (div & 0xFFF0) | ((div >> 1) & 0x7); /* int[15:4], frac[2:0] */
        if (div > 0xFFF7) /* Limit maximum div = 255.125, if div over max */
                div = 0xFFF7;
        else if (!div)    /* Limit div = 0.125, if div is zero */
                div = 0xFFF0 & (div << 4);
        USART_REG->BRR = div;

        /* Enable USART */
        USART_REG->CR1 |= USART_CR1_UE;

        /* Enable RX DMA */
        DMA2_Stream5->CR |= DMA_SxCR_EN;

        /* Clear TC bit in the SR */
        USART_REG->SR &= ~USART_SR_TC;

        /* Select three sample bit method and Enable DMA TX/RX */
        USART_REG->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;

}

void USART_DMA_transmit(char *buffer, uint16_t size)
{
        /* Set memory address */
        DMA2_Stream7->M0AR = (uint32_t)buffer;

        /* Set number of data items to transfer */
        DMA2_Stream7->NDTR = (uint32_t)size;

        /* Read SR to clear TC and Check if last transmission is not done */
        while (!(USART_REG->SR & USART_SR_TC));

        /* Enable DMA */
        DMA2_Stream7->CR |= DMA_SxCR_EN;

        tx_finish = 0;
}

void DMA2_Stream5_IRQHandler(void)
{
        DMA2->HIFCR |= DMA_HISR_TCIF5; /* Clear TCIF */
        rx_finish = 1;
}


void DMA2_Stream7_IRQHandler(void)
{
        DMA2->HIFCR |= DMA_HISR_TCIF7; /* Clear TCIF */

        /* Moving data to DR is completed, but transmit is not done.
         * User need to check TC flag by software.
         */
        tx_finish = 1;
}

