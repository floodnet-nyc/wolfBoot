/* author: p@FloodNet */

#include <ctype.h>
#include <stdint.h>

/* rev-1 debug pins are on PC0, and PC1 - LPUART1 */

#define LPUART1 	 	0x46002400
#define LPUART1_PIN_AF 		8
#define LPUART1_RX_PIN 		0
#define LPUART1_TX_PIN 		1

#define LPUART1_CR1 	(*(volatile uint32_t *)(LPUART1))
#define LPUART1_CR2 	(*(volatile uint32_t *)(LPUART1 + 0x04))
#define LPUART1_CR3 	(*(volatile uint32_t *)(LPUART1 + 0x08))
#define LPUART1_BRR 	(*(volatile uint32_t *)(LPUART1 + 0x0c))
#define LPUART1_RQR 	(*(volatile uint32_t *)(LPUART1 + 0x18))
#define LPUART1_ISR 	(*(volatile uint32_t *)(LPUART1 + 0x1c))
#define LPUART1_ICR 	(*(volatile uint32_t *)(LPUART1 + 0x20))
#define LPUART1_RDR 	(*(volatile uint32_t *)(LPUART1 + 0x24))
#define LPUART1_TDR 	(*(volatile uint32_t *)(LPUART1 + 0x28))
#define LPUART1_PRESC 	(*(volatile uint32_t *)(LPUART1 + 0x2c))
#define LPUART1_AUTOCR 	(*(volatile uint32_t *)(LPUART1 + 0x30))

#define LPUART1_CR1_ENABLE 		(1 << 0)
#define LPUART1_CR1_RX_ENABLE 		(1 << 2)
#define LPUART1_CR1_TX_ENABLE 		(1 << 3)
#define LPUART1_CR1_PARITY_SEL 		(1 << 9)
#define LPUART1_CR1_PCTRL_ENABLE 	(1 << 10)
#define LPUART1_CR1_WORD_LEN_M0 	(1 << 12)
#define LPUART1_CR1_WORD_LEN_M1 	(1 << 28)
#define LPUART1_CR1_FIFO_ENABLE 	(1 << 29)
#define LPUART1_CR2_STOPBITS 		(3 << 12)
#define LPUART1_ISR_RXNE  		(1 << 5)
#define LPUART1_ISR_TX_COMPLETE 	(1 << 6)
#define LPUART1_ISR_TXE 		(1 << 7)

#define LPUART1_CLOCK_SPEED 		(100 * 1000000) /* 100 MHz */

#define NS_RCC_REG_MAP			0x46020C00

/* LPUART1 clocks */
#define APB3_LPUART_RESET 		(1 << 6)
#define LPUART1_RCC_APB3RSTR 		(*(volatile uint32_t *)(NS_RCC_REG_MAP + 0x080))

/* GPIOC in RCC is in AHB2 */
#define RCC_AHB2ENR1 	 		(*(volatile uint32_t *)(NS_RCC_REG_MAP + 0x08C))
#define GPIOC_RCC_AHB2ENR1_ENABLE  	(1 << 2)	

#define GPIOC_BASE_ADDRESS 		0x42020800
#define GPIOC_MODE 			(*(volatile uint32_t *)(GPIOC_BASE_ADDRESS))
#define GPIOC_AFRL 			(*(volatile uint32_t *)(GPIOC_BASE_ADDRESS + 0x20))
#define GPIOC_AFRH 			(*(volatile uint32_t *)(GPIOC_BASE_ADDRESS + 0x24))
#define GPIOC_AF_MODE 			2


static void uart_pins_setup(void) {
	uint32_t reg;

	/* enable GPIOC clock in RCC */
	RCC_AHB2ENR1 |= GPIOC_RCC_AHB2ENR1_ENABLE;
	
	/* get current mode of GPIOC but clear RX pin fileds */
	reg = GPIOC_MODE & ~(0x03 << (LPUART1_RX_PIN * 2)); 
	/* set RX pin mode as alternate function */
	GPIOC_MODE = reg | (2 << (LPUART1_RX_PIN * 2));
	/* get current mode of GPIOC but clear TX pin fields */
	reg = GPIOC_MODE & ~(0x03 << (LPUART1_TX_PIN * 2)); 
	/* set RX pin mode as alternate function */
	GPIOC_MODE = reg | (2 << (LPUART1_TX_PIN * 2));

	/* Alternate function: use low pins (0 and 1) */
	/* get current values of LOW AF of GPIOC */
	reg = GPIOC_AFRL & ~(0xf << (LPUART1_TX_PIN * 4));
	/* set TX pin function as AF8 - from datasheet AF8 is LPUART */	
	GPIOC_AFRL = reg | (LPUART1_PIN_AF << (LPUART1_TX_PIN * 4));
	/* get current values of LOW AF of GPIOC */
	reg = GPIOC_AFRL & ~(0xf << ((LPUART1_RX_PIN) * 4));
	/* set RX pin function as AF8 */
	GPIOC_AFRL = reg | (LPUART1_PIN_AF << ((LPUART1_RX_PIN) * 4));
}

int uart_tx(const uint8_t c) {

	uint32_t reg;

	do{
		reg = LPUART1_ISR;
	} while((reg & LPUART1_ISR_TXE) == 0);
	LPUART1_TDR = c;
	return 0;
}

int uart_rx(uint8_t *c) {

	volatile uint32_t reg;

	reg = LPUART1_ISR;
	if((reg & LPUART1_ISR_RXNE) != 0) {
		reg = LPUART1_RDR;
		*c = (uint8_t)(reg & 0xFF);
		return 1;
	}
	return 0;
}

int uart_init(uint32_t bitrate, uint8_t data, char parity, uint8_t stop) {

	uint32_t reg;

	/* enable GPIO pins */
	uart_pins_setup();
	/* on reset, LPUART FIFO mode is disabled, so we use alternate function (AF)*/
	/* Reset LPUART1 and disable it */
	LPUART1_RCC_APB3RSTR |= APB3_LPUART_RESET;
	LPUART1_CR1 &= ~(LPUART1_CR1_ENABLE);

	/* Configure TX and RX */
	LPUART1_CR1 |= (LPUART1_CR1_TX_ENABLE | LPUART1_CR1_RX_ENABLE);

	/* Configure clock */
	LPUART1_BRR |= (uint16_t)(LPUART1_CLOCK_SPEED/bitrate);

	/* Configure data bits */
	if(data == 8) {
		LPUART1_CR1 &= ~(LPUART1_CR1_WORD_LEN_M1);
		LPUART1_CR1 &= ~(LPUART1_CR1_WORD_LEN_M0);
	} else if(data == 9) {
		LPUART1_CR1 &= ~(LPUART1_CR1_WORD_LEN_M1);
		LPUART1_CR1 |= LPUART1_CR1_WORD_LEN_M0;
	} else if(data == 7){
		LPUART1_CR1 |= LPUART1_CR1_WORD_LEN_M1;
		LPUART1_CR1 &= ~(LPUART1_CR1_WORD_LEN_M0);
	} else {
		return -1;
	}

	/* configure parity */
	switch (parity) {
		case 'O':
			/* enable parity control */
			LPUART1_CR1 |= LPUART1_CR1_PCTRL_ENABLE; 		
			LPUART1_CR1 |= LPUART1_CR1_PARITY_SEL; 
			break;
		case 'E':
			/* enable parity control */
			LPUART1_CR1 |= LPUART1_CR1_PCTRL_ENABLE; 			
			LPUART1_CR1 &= ~(LPUART1_CR1_PARITY_SEL);	
			break;
		default:
			/* disable parity control and go with even */
			LPUART1_CR1 &= ~(LPUART1_CR1_PCTRL_ENABLE | LPUART1_CR1_PARITY_SEL);
			break;
	}
	/* set stop bits */
	reg = LPUART1_CR2 & (~LPUART1_CR2_STOPBITS);
	if(stop > 1) {
		LPUART1_CR2 = reg & (2 << 12);
	} else {
		LPUART1_CR2 = reg;
	}

	/* turn on LPUART1 */
	LPUART1_CR1 |= LPUART1_CR1_ENABLE;
	return 0;
}
