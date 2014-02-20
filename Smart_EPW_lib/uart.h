


/*the usart acept the command from RX when RX interrupt is trigger*/
unsigned char Receive_data ;




/*Setting the USART MAX string lenth */
#define MAX_STRLEN 2 // this is the maximum string length of our string in characters
volatile unsigned char received_string[MAX_STRLEN]; // this will hold the recieved string




void init_USART3(uint32_t baurate);
void USART3_IRQHandler(void);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
