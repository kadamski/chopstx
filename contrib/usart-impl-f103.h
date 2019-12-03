/* Hardware registers */
struct USART {
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t BRR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t CR3;
  volatile uint32_t GTPR;
};

#define USART_SR_CTS	(1 << 9)
#define USART_SR_LBD	(1 << 8)
#define USART_SR_TXE	(1 << 7)
#define USART_SR_TC	(1 << 6)
#define USART_SR_RXNE	(1 << 5)
#define USART_SR_IDLE	(1 << 4)
#define USART_SR_ORE	(1 << 3)
#define USART_SR_NE	(1 << 2)
#define USART_SR_FE	(1 << 1)
#define USART_SR_PE	(1 << 0)


#define USART_CR1_UE		(1 << 13)
#define USART_CR1_M		(1 << 12)
#define USART_CR1_WAKE		(1 << 11)
#define USART_CR1_PCE		(1 << 10)
#define USART_CR1_PS		(1 <<  9)
#define USART_CR1_PEIE		(1 <<  8)
#define USART_CR1_TXEIE		(1 <<  7)
#define USART_CR1_TCIE		(1 <<  6)
#define USART_CR1_RXNEIE	(1 <<  5)
#define USART_CR1_IDLEIE	(1 <<  4)
#define USART_CR1_TE		(1 <<  3)
#define USART_CR1_RE		(1 <<  2)
#define USART_CR1_RWU		(1 <<  1)
#define USART_CR1_SBK		(1 <<  0)

#define USART_CR3_CTSE		(1 <<  9)
#define USART_CR3_RTSE		(1 <<  8)
#define USART_CR3_SCEN		(1 <<  5)
#define USART_CR3_NACK		(1 <<  4)
