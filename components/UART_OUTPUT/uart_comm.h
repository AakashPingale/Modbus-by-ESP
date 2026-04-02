#ifndef UART_COMM_H
#define UART_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

void uart_comm_printf(const char *fmt, ...);
void uart_comm_init(void);
void uart_comm_set_callback(void (*cb)(char *cmd));

#ifdef __cplusplus
}
#endif

#endif // UART_COMM_H
