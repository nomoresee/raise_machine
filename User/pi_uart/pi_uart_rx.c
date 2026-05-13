#include "pi_uart_rx.h"

#include "main.h"
#include "usart.h"
#include <string.h>

#define PI_UART_RX_CHUNK 160

static uint8_t s_rx_chunk[PI_UART_RX_CHUNK];
static char s_line_acc[256];
static size_t s_line_len;
static char s_last_line[256];
static volatile uint8_t s_has_new_line;

static void pi_uart_rx_restart(void)
{
  (void)HAL_UARTEx_ReceiveToIdle_IT(&huart7, s_rx_chunk, PI_UART_RX_CHUNK);
}

void pi_uart_rx_init(void)
{
  s_line_len = 0;
  s_has_new_line = 0U;
  memset(s_last_line, 0, sizeof(s_last_line));
  memset(s_line_acc, 0, sizeof(s_line_acc));
  pi_uart_rx_restart();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != UART7)
  {
    return;
  }

  for (uint16_t i = 0; i < Size; i++)
  {
    uint8_t c = s_rx_chunk[i];
    if (c == '\r' || c == '\n')
    {
      if (s_line_len > 0U)
      {
        s_line_acc[s_line_len] = '\0';
        (void)strncpy(s_last_line, s_line_acc, sizeof(s_last_line) - 1U);
        s_last_line[sizeof(s_last_line) - 1U] = '\0';
        s_has_new_line = 1U;
        s_line_len = 0U;
      }
    }
    else if (s_line_len < (sizeof(s_line_acc) - 1U))
    {
      if (c >= 32U && c < 127U)
      {
        s_line_acc[s_line_len++] = (char)c;
      }
    }
  }

  pi_uart_rx_restart();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != UART7)
  {
    return;
  }
  pi_uart_rx_restart();
}

void pi_uart_rx_peek_line(char *dst, size_t dst_sz)
{
  if (dst == NULL || dst_sz == 0U)
  {
    return;
  }
  (void)strncpy(dst, s_last_line, dst_sz - 1U);
  dst[dst_sz - 1U] = '\0';
}

uint8_t pi_uart_rx_take_new_line(char *dst, size_t dst_sz)
{
  if (dst == NULL || dst_sz == 0U || s_has_new_line == 0U)
  {
    return 0U;
  }
  (void)strncpy(dst, s_last_line, dst_sz - 1U);
  dst[dst_sz - 1U] = '\0';
  s_has_new_line = 0U;
  return 1U;
}
