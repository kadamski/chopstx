#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>
#include <contrib/usart.h>
#include <stdarg.h>

extern int vsnprintf(char *str, size_t size, const char *format, va_list ap);

#define USART_NO 0

/* Intentionally only adding LF */
void
debug (const char *fmt, ...)
{
  char buf[256];
  va_list ap;
  size_t len;

  va_start(ap, fmt);
  len = vsnprintf (buf, sizeof buf - 1, fmt, ap);
  va_end(ap);
  if (len >= sizeof buf - 1)
    len = sizeof buf - 2;
  buf[len++] = '\n';
  usart_write (USART_NO, buf, len);
}
