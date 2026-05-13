#ifndef PI_UART_RX_H
#define PI_UART_RX_H

#include <stddef.h>
#include <stdint.h>

void pi_uart_rx_init(void);

/** 复制当前缓存的最后一行（无新数据也可读），用于周期刷新 LCD */
void pi_uart_rx_peek_line(char *dst, size_t dst_sz);

/** 若有新的一行（相对上次 take），复制到 dst 并返回 1 */
uint8_t pi_uart_rx_take_new_line(char *dst, size_t dst_sz);

#endif
