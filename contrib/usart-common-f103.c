static void
usart_config_recv_enable (struct USART *USARTx, int on)
{
  if (on)
    USARTx->CR1 |= USART_CR1_RE;
  else
    USARTx->CR1 &= ~USART_CR1_RE;
}


int
usart_config (uint8_t dev_no, uint32_t config_bits)
{
  struct USART *USARTx = get_usart_dev (dev_no);
  uint8_t baud_spec = (config_bits & MASK_BAUD);
  int i;
  uint32_t cr1_config = (USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TE);
				/* TXEIE/TCIE will be enabled when
				   putting char */
				/* No CTSIE, PEIE, IDLEIE, LBDIE */
  if (USARTx == NULL)
    return -1;

  /* Disable USART before configure.  */
  USARTx->CR1 &= ~USART_CR1_UE;

  if (((config_bits & MASK_CS) == CS7 && (config_bits & PARENB))
      || ((config_bits & MASK_CS) == CS8 && (config_bits & PARENB) == 0))
    cr1_config &= ~USART_CR1_M;
  else if ((config_bits & MASK_CS) == CS8)
    cr1_config |=  USART_CR1_M;
  else
    return -1;

  if ((config_bits & PARENB) == 0)
    cr1_config &= ~(USART_CR1_PCE | USART_CR1_PEIE);
  else
    cr1_config |=  (USART_CR1_PCE | USART_CR1_PEIE);

  if ((config_bits & PARODD) == 0)
    cr1_config &= ~USART_CR1_PS;
  else
    cr1_config |=  USART_CR1_PS;

  if ((config_bits & MASK_STOP) == STOP0B5)
    USARTx->CR2 = (0x1 << 12);
  else if ((config_bits & MASK_STOP) == STOP1B)
    USARTx->CR2 = (0x0 << 12);
  else if ((config_bits & MASK_STOP) == STOP1B5)
    USARTx->CR2 = (0x3 << 12);
  else /* if ((config_bits & MASK_STOP) == STOP2B) */
    USARTx->CR2 = (0x2 << 12);

  for (i = 0; i < NUM_BAUD; i++)
    if (brr_table[i].baud_spec == baud_spec)
      break;

  if (i >= NUM_BAUD)
    return -1;

  USARTx->BRR = brr_table[i].brr_value;

  if ((config_bits & MASK_FLOW))
    USARTx->CR3 = USART_CR3_CTSE | USART_CR3_RTSE;
  else
    USARTx->CR3 = 0;

  if (!(config_bits & MASK_MODE))
    cr1_config |= USART_CR1_RE;

  USARTx->CR1 = cr1_config;

  /* SCEN (smartcard enable) should be set _after_ CR1.  */
  if ((config_bits & MASK_MODE))
    {
      if ((config_bits & MASK_MODE) == MODE_SMARTCARD)
	{
	  USARTx->GTPR = (1 << 8) | 5;
	  USARTx->CR3 |= (USART_CR3_SCEN | USART_CR3_NACK);
	}
      else if ((config_bits & MASK_MODE) == MODE_IRDA)
	USARTx->CR3 |= (1 << 1);
      else if ((config_bits & MASK_MODE) == MODE_IRDA_LP)
	USARTx->CR3 |= (1 << 2) | (1 << 1);
    }

  return 0;
}


void
usart_init0 (int (*cb) (uint8_t dev_no, uint16_t notify_bits))
{
  int i;

  ss_notify_callback = cb;

  for (i = 0; i < NUM_USART; i++)
    {
      if (usart_array[i].stat)
	usart_array[i].stat->dev_no = i + USART_DEVNO_START;
      chopstx_claim_irq (usart_array[i].intr, usart_array[i].irq_num);
    }

  usart_rcc_setup ();
}

#define UART_STATE_BITMAP_RX_CARRIER (1 << 0)
#define UART_STATE_BITMAP_TX_CARRIER (1 << 1)
#define UART_STATE_BITMAP_BREAK      (1 << 2)
#define UART_STATE_BITMAP_RINGSIGNAL (1 << 3)
#define UART_STATE_BITMAP_FRAMING    (1 << 4)
#define UART_STATE_BITMAP_PARITY     (1 << 5)
#define UART_STATE_BITMAP_OVERRUN    (1 << 6)

static int
handle_intr (struct USART *USARTx, struct rb *rb2a, struct usart_stat *stat)
{
  int tx_ready = 0;
  uint32_t r = USARTx->SR;
  int notify_bits = 0;
  int smartcard_mode = ((USARTx->CR3 & USART_CR3_SCEN) != 0);

  if (smartcard_mode)
    {
      if ((r & USART_SR_TC))
	{
	  tx_ready = 1;
	  USARTx->CR1 &= ~USART_CR1_TCIE;
	}
    }
  else
    {
      if ((r & USART_SR_TXE))
	{
	  tx_ready = 1;
	  USARTx->CR1 &= ~USART_CR1_TXEIE;
	}
    }

  if ((r & USART_SR_RXNE))
    {
      uint32_t data = USARTx->DR;

      /* DR register should be accessed even if data is not used.
       * Its read-access has side effect of clearing error flags.
       */
      asm volatile ("" : : "r" (data) : "memory");

      if ((r & USART_SR_NE))
	stat->err_rx_noise++;
      else if ((r & USART_SR_FE))
	{
	  /* NOTE: Noway to distinguish framing error and break  */

	  stat->rx_break++;
	  notify_bits |= UART_STATE_BITMAP_BREAK;
	}
      else if ((r & USART_SR_PE))
	{
	  stat->err_rx_parity++;
	  notify_bits |= UART_STATE_BITMAP_PARITY;
	}
      else
	{
	  if ((r & USART_SR_ORE))
	    {
	      stat->err_rx_overrun++;
	      notify_bits |= UART_STATE_BITMAP_OVERRUN;
	    }

	  /* XXX: if CS is 7-bit, mask it, or else parity bit in upper layer */
	  if (rb_ll_put (rb2a, (data & 0xff)) < 0)
	    stat->err_rx_overflow++;
	  else
	    stat->rx++;
	}
    }
  else if ((r & USART_SR_ORE))
    {				/* Clear ORE */
      uint32_t data = USARTx->DR;
      asm volatile ("" : : "r" (data) : "memory");
      stat->err_rx_overrun++;
      notify_bits |= UART_STATE_BITMAP_OVERRUN;
    }

  if (notify_bits)
    {
      if (ss_notify_callback
	  && (*ss_notify_callback) (stat->dev_no, notify_bits))
	stat->err_notify_overflow++;
    }

  return tx_ready;
}

static int
handle_tx (struct USART *USARTx, struct rb *rb2h, struct usart_stat *stat)
{
  int tx_ready = 1;
  int c = rb_ll_get (rb2h);

  if (c >= 0)
    {
      uint32_t r;
      int smartcard_mode = ((USARTx->CR3 & USART_CR3_SCEN) != 0);

      USARTx->DR = (c & 0xff);
      stat->tx++;
      r = USARTx->SR;
      if (smartcard_mode)
	{
	  if ((r & USART_SR_TC) == 0)
	    {
	      tx_ready = 0;
	      USARTx->CR1 |= USART_CR1_TCIE;
	    }
	}
      else
	{
	  if ((r & USART_SR_TXE) == 0)
	    {
	      tx_ready = 0;
	      USARTx->CR1 |= USART_CR1_TXEIE;
	    }
	}
    }

  return tx_ready;
}

int
usart_send_break (uint8_t dev_no)
{
  struct USART *USARTx = get_usart_dev (dev_no);
  if (USARTx == NULL)
    return -1;

  if ((USARTx->CR1 & 0x01))
    return 1;	/* Busy sending break, which was requested before.  */

  USARTx->CR1 |= 0x01;
  return 0;
}

int
usart_block_sendrecv (uint8_t dev_no, const char *s_buf, uint16_t s_buflen,
		      char *r_buf, uint16_t r_buflen,
		      uint32_t *timeout_block_p, uint32_t timeout_char)
{
  uint32_t timeout;
  uint8_t *p;
  int len;
  uint32_t r;
  uint32_t data;
  struct USART *USARTx = get_usart_dev (dev_no);
  int smartcard_mode = ((USARTx->CR3 & USART_CR3_SCEN) != 0);
  struct chx_intr *usartx_intr = get_usart_intr (dev_no);
  struct chx_poll_head *ph[1];

  if (usartx_intr == NULL)
    return -1;

  ph[0] = (struct chx_poll_head *)usartx_intr;

  p = (uint8_t *)s_buf;
  if (p)
    {
      if (smartcard_mode)
	usart_config_recv_enable (USARTx, 0);

      USARTx->CR1 |= USART_CR1_TXEIE;

      /* Sending part */
      while (1)
	{
	  chopstx_poll (NULL, 1, ph);

	  r = USARTx->SR;

	  /* Here, ignore recv error(s).  */
	  if ((r & USART_SR_RXNE) || (r & USART_SR_ORE))
	    {
	      data = USARTx->DR;
	      asm volatile ("" : : "r" (data) : "memory");
	    }

	  if ((r & USART_SR_TXE))
	    {
	      if (s_buflen == 0)
		break;
	      else
		{
		  /* Keep TXEIE bit */
		  USARTx->DR = *p++;
		  s_buflen--;
		}
	    }

	  chopstx_intr_done (usartx_intr);
	}

      USARTx->CR1 &= ~USART_CR1_TXEIE;
      if (smartcard_mode)
	{
	  if (timeout_block_p && (*timeout_block_p))
	    do
	      r = USARTx->SR;
	    while (((r & USART_SR_TC) == 0));

	  usart_config_recv_enable (USARTx, 1);

	  if (timeout_block_p && *timeout_block_p == 0)
	    {
	      /* Ignoring the echo back.  */
	      do
		r = USARTx->SR;
	      while (((r & USART_SR_TC) == 0));

	      if ((r & USART_SR_RXNE))
		{
		  data = USARTx->DR;
		  asm volatile ("" : : "r" (data) : "memory");
		}

	      *timeout_block_p = timeout_char;
	    }
	}

      chopstx_intr_done (usartx_intr);
    }

  if (r_buf == NULL)
    return 0;

  if (!p)
    if (smartcard_mode)
      usart_config_recv_enable (USARTx, 1);

  /* Receiving part */
  r = chopstx_poll (timeout_block_p, 1, ph);
  if (r == 0)
    return 0;

  p = (uint8_t *)r_buf;
  len = 0;

  while (1)
    {
      r = USARTx->SR;

      data = USARTx->DR;
      asm volatile ("" : : "r" (data) : "memory");

      if ((r & USART_SR_RXNE))
	{
	  if ((r & USART_SR_NE) || (r & USART_SR_FE) || (r & USART_SR_PE))
	    /* ignore error, for now.  XXX: ss_notify */
	    ;
	  else
	    {
	      *p++ = (data & 0xff);
	      len++;
	      r_buflen--;
	      if (r_buflen == 0)
		{
		  chopstx_intr_done (usartx_intr);
		  break;
		}
	    }
	}
      else if ((r & USART_SR_ORE))
	{
	  data = USARTx->DR;
	  asm volatile ("" : : "r" (data) : "memory");
	}

      chopstx_intr_done (usartx_intr);
      timeout = timeout_char;
      r = chopstx_poll (&timeout, 1, ph);
      if (r == 0)
	break;
    }

  return len;
}
