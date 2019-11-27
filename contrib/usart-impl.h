struct usart {
  struct USART *USART;
  struct chx_intr *intr;
  uint8_t irq_num;
  struct usart_stat *stat;
  struct rb *rb_a2h;
  struct rb *rb_h2a;
  uint8_t *buf_a2h;
  uint8_t *buf_h2a;
  chopstx_poll_cond_t *app_write_event;
  int *tx_ready;
};


static int handle_intr (struct USART *USARTx, struct rb *rb2a, struct usart_stat *stat);
static int handle_tx (struct USART *USARTx, struct rb *rb2h, struct usart_stat *stat);
static void usart_config_recv_enable (struct USART *USARTx, int on);

struct brr_setting {
  uint8_t baud_spec;
  uint32_t brr_value;
};
#define NUM_BAUD (int)(sizeof (brr_table) / sizeof (struct brr_setting))

#define NUM_USART ((int)(sizeof (usart_array) / sizeof (struct usart)))
